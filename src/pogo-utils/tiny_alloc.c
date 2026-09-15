/**
 * @file tiny_alloc.c
 * @brief Implementation of a tiny segregated free-list allocator (alignment-safe).
 */
#include "tiny_alloc.h"
#include <string.h>
#include <stdint.h>

/* ------------------------------------------------------------------------- */
/* Internal header at the start of each slot                                 */
typedef struct __attribute__((packed)) {
    uint16_t class_idx; /* which size class this slot belongs to */
    uint16_t flags;     /* allocation state marker */
} ta_hdr_t;

/* Nonzero markers make cleared or accidentally overwritten headers invalid. */
#define TA_FLAG_FREE      UINT16_C(0x4652)
#define TA_FLAG_ALLOCATED UINT16_C(0xA110)

/* Alignment helpers */
static inline size_t ta_align(void) { return sizeof(void*); }
static int round_up_sz(size_t x, size_t a, size_t *result) {
    if (!result || a == 0u) return 0;
    size_t r = x % a;
    size_t add = r ? (a - r) : 0u;
    if (x > SIZE_MAX - add) return 0;
    *result = x + add;
    return 1;
}
/* Distance from slot base to user payload (and to free-list "next" when FREE) */
static inline size_t hdr_off(void) {
    size_t a = sizeof(void*);
    size_t s = sizeof(ta_hdr_t);
    size_t r = s % a;
    return r ? (s + (a - r)) : s;
}

/* where we keep the free-list "next" pointer (inside payload area) */
static inline void **next_ptr(uint8_t *slot_base) {
    return (void **)(slot_base + hdr_off());
}

static inline void *get_next(uint8_t *slot_base) {
    return *next_ptr(slot_base);
}

static inline void set_next(uint8_t *slot_base, void *n) {
    *next_ptr(slot_base) = n;
}

static inline void *user_from_slot(uint8_t *slot_base) {
    return (void *)(slot_base + hdr_off());
}

static inline void push_free(void **head, uint8_t *slot_base) {
    set_next(slot_base, *head);
    *head = slot_base;
}

/* Validate immutable allocator metadata before using it for pointer walks. */
static int allocator_layout_valid(const tiny_alloc_t *ta) {
    if (!ta || ta->num_classes == 0u ||
        ta->num_classes > TINY_ALLOC_MAX_CLASSES ||
        !ta->heap_begin || !ta->heap_end) {
        return 0;
    }

    const uintptr_t begin = (uintptr_t)ta->heap_begin;
    const uintptr_t end = (uintptr_t)ta->heap_end;
    if (end < begin || begin % ta_align() != 0u || end % ta_align() != 0u) {
        return 0;
    }

    for (uint8_t c = 0; c < ta->num_classes; ++c) {
        const size_t payload = ta->class_sz[c] < sizeof(void*)
            ? sizeof(void*) : ta->class_sz[c];
        const size_t slot = ta->slot_sz[c];
        const size_t header_bytes = hdr_off();
        size_t expected_slot = 0u;
        if (ta->class_sz[c] == 0u ||
            payload > SIZE_MAX - header_bytes ||
            !round_up_sz(header_bytes + payload, ta_align(), &expected_slot) ||
            expected_slot > UINT16_MAX || slot != expected_slot ||
            (c > 0u && ta->class_sz[c] <= ta->class_sz[c - 1u])) {
            return 0;
        }
    }
    return 1;
}

/* Replay the deterministic round-robin carving order to prove that a pointer
 * is an exact slot boundary. This avoids trusting user-adjacent header bytes. */
static int locate_slot_base(const tiny_alloc_t *ta, const void *candidate,
                            uint16_t *class_out) {
    if (!candidate || !allocator_layout_valid(ta)) return 0;

    uintptr_t p = (uintptr_t)ta->heap_begin;
    const uintptr_t end = (uintptr_t)ta->heap_end;
    const uintptr_t wanted = (uintptr_t)candidate;
    if (wanted < p || wanted >= end) return 0;

    while (p < end) {
        int made_progress = 0;
        for (uint16_t c = 0; c < ta->num_classes; ++c) {
            const size_t slot = ta->slot_sz[c];
            if ((size_t)(end - p) >= slot) {
                if (p == wanted) {
                    if (class_out) *class_out = c;
                    return 1;
                }
                p += slot;
                made_progress = 1;
            }
        }
        if (!made_progress) break;
    }
    return 0;
}

/* Convert only after checking address arithmetic, then validate the boundary. */
static int locate_user_slot(const tiny_alloc_t *ta, const void *user_ptr,
                            uint8_t **slot_out, uint16_t *class_out) {
    if (!user_ptr || !allocator_layout_valid(ta)) return 0;

    const uintptr_t begin = (uintptr_t)ta->heap_begin;
    const uintptr_t user = (uintptr_t)user_ptr;
    const size_t offset = hdr_off();
    if (user < begin || user - begin < offset) return 0;

    uint8_t *slot = (uint8_t *)(user - offset);
    uint16_t c = 0u;
    if (!locate_slot_base(ta, slot, &c)) return 0;
    if (slot_out) *slot_out = slot;
    if (class_out) *class_out = c;
    return 1;
}

/* Pop only a valid FREE slot. On corruption, detach the unsafe list rather
 * than following an unchecked pointer into arbitrary memory. */
static uint8_t *pop_free(tiny_alloc_t *ta, uint16_t expected_class) {
    uint8_t *slot = (uint8_t *)ta->free_head[expected_class];
    if (!slot) return NULL;

    uint16_t actual_class = 0u;
    if (!locate_slot_base(ta, slot, &actual_class) ||
        actual_class != expected_class) {
        ta->free_head[expected_class] = NULL;
        return NULL;
    }

    ta_hdr_t *header = (ta_hdr_t *)slot;
    if (header->class_idx != expected_class || header->flags != TA_FLAG_FREE) {
        ta->free_head[expected_class] = NULL;
        return NULL;
    }

    ta->free_head[expected_class] = get_next(slot);
    header->flags = TA_FLAG_ALLOCATED;
    return slot;
}

/* Choose class index for requested payload */
static int choose_class(const tiny_alloc_t *ta, size_t nbytes) {
    for (int i = 0; i < ta->num_classes; ++i)
        if (nbytes <= ta->class_sz[i]) return i;
    return -1;
}

void tiny_alloc_init(tiny_alloc_t *ta,
                     void *heap_ptr, size_t heap_bytes,
                     const uint16_t *classes, uint8_t num_classes)
{
    if (!ta) return;
    memset(ta, 0, sizeof(*ta));
    if (!heap_ptr || heap_bytes < 32u) return;

    /* defaults covering 16..192 */
    static const uint16_t k_default[] = { 16, 32, 64, 96, 128, 192 };
    if (!classes || !num_classes) {
        classes = k_default;
        num_classes = (uint8_t)(sizeof(k_default)/sizeof(k_default[0]));
    }
    if (num_classes > TINY_ALLOC_MAX_CLASSES) return;

    /* Pointer-align the heap using checked integer address arithmetic. */
    const size_t alignment = ta_align();
    const uintptr_t raw_address = (uintptr_t)heap_ptr;
    const size_t remainder = (size_t)(raw_address % alignment);
    const size_t drop = remainder ? (alignment - remainder) : 0u;
    if (drop >= heap_bytes || raw_address > UINTPTR_MAX - drop) return;

    const uintptr_t aligned_address = raw_address + drop;
    heap_bytes -= drop;
    if (heap_bytes > UINTPTR_MAX - aligned_address) return;

    uint8_t *h = (uint8_t *)aligned_address;
    const uintptr_t end_address = aligned_address + heap_bytes;

    /* Validate class ordering and reject sizes that cannot be represented by
     * the uint16_t slot metadata. A bad table leaves the allocator disabled. */
    for (uint8_t i = 0; i < num_classes; ++i) {
        if (classes[i] == 0u || (i > 0u && classes[i] <= classes[i - 1u])) {
            memset(ta, 0, sizeof(*ta));
            return;
        }
        ta->class_sz[i] = classes[i];
        /* Slot needs: header + max(sizeof(void*) for free-list, payload for user) */
        size_t payload = (size_t)classes[i];
        if (payload < sizeof(void*)) payload = sizeof(void*);  /* need room for free-list ptr */
        const size_t header_bytes = hdr_off();
        size_t slot = 0u;
        if (payload > SIZE_MAX - header_bytes ||
            !round_up_sz(header_bytes + payload, alignment, &slot) ||
            slot > UINT16_MAX) {
            memset(ta, 0, sizeof(*ta));
            return;
        }
        ta->slot_sz[i] = (uint16_t)slot;
    }

    ta->num_classes = num_classes;
    ta->heap_begin = h;

    /* carve fairly across classes (round-robin) so each class gets slots */
    uintptr_t p = aligned_address;

    /* find the smallest slot size; if we can't fit this, we're done */
    size_t min_slot = ta->slot_sz[0];
    for (uint8_t i = 1; i < num_classes; ++i)
        if (ta->slot_sz[i] < min_slot) min_slot = ta->slot_sz[i];

    while ((size_t)(end_address - p) >= min_slot) {
        int made_progress = 0;
        for (uint8_t cls = 0; cls < num_classes; ++cls) {
            size_t slot = ta->slot_sz[cls];
            if ((size_t)(end_address - p) >= slot) {
                uint8_t *slot_base = (uint8_t *)p;
                ta_hdr_t *hdrp = (ta_hdr_t *)slot_base;
                hdrp->class_idx = cls;
                hdrp->flags = TA_FLAG_FREE;
                push_free(&ta->free_head[cls], slot_base);
                p += slot;
                made_progress = 1;
            }
        }
        if (!made_progress) break; /* leftover < any slot size */
    }
    ta->heap_end = (uint8_t *)p;
}

void *tiny_malloc(tiny_alloc_t *ta, size_t nbytes) {
    if (!allocator_layout_valid(ta) || nbytes == 0u) return NULL;
    int cls = choose_class(ta, nbytes);
    if (cls < 0) return NULL;
    for (int c = cls; c < ta->num_classes; ++c) {
        uint8_t *slot = pop_free(ta, (uint16_t)c);
        if (slot) {
            /* header is at slot, payload starts at slot+hdr_off() */
            ta_hdr_t *h = (ta_hdr_t *)slot;
            h->class_idx = (uint16_t)c;
            return user_from_slot(slot);
        }
    }
    return NULL;
}

void *tiny_calloc(tiny_alloc_t *ta, size_t count, size_t size) {
    if (!ta) return NULL;
    if (count == 0u || size == 0u || size > SIZE_MAX / count) return NULL;
    size_t need = count * size;
    void *p = tiny_malloc(ta, need);
    if (p) memset(p, 0, need);
    return p;
}

void tiny_free(tiny_alloc_t *ta, void *ptr) {
    if (!ta || !ptr) return;

    uint8_t *slot = NULL;
    uint16_t c = 0u;
    if (!locate_user_slot(ta, ptr, &slot, &c)) return;

    ta_hdr_t *h = (ta_hdr_t *)slot;
    if (h->class_idx != c || h->flags != TA_FLAG_ALLOCATED) return;

    /* Mark FREE before linking. A repeated free now fails the state check. */
    h->flags = TA_FLAG_FREE;
    push_free(&ta->free_head[c], slot);
}

void *tiny_realloc(tiny_alloc_t *ta, void *ptr, size_t new_size) {
    if (!ta) return NULL;
    if (!ptr) return tiny_malloc(ta, new_size);
    if (new_size == 0) { tiny_free(ta, ptr); return NULL; }

    uint8_t *slot = NULL;
    uint16_t expected_class = 0u;
    if (!locate_user_slot(ta, ptr, &slot, &expected_class)) return NULL;

    ta_hdr_t *h = (ta_hdr_t *)slot;
    if (h->class_idx != expected_class || h->flags != TA_FLAG_ALLOCATED) {
        return NULL;
    }
    int old_c = expected_class;
    size_t old_payload = ta->class_sz[old_c];

    int new_c = choose_class(ta, new_size);
    if (new_c < 0) return NULL;

    /* Fast path: fits in the same class */
    if (new_c == old_c) return ptr;

    /* Allocate new, copy, free old */
    void *np = tiny_malloc(ta, new_size);
    if (!np) return NULL;
    size_t n = old_payload < new_size ? old_payload : new_size;
    memcpy(np, ptr, n);
    tiny_free(ta, ptr);
    return np;
}

size_t tiny_usable_size(tiny_alloc_t *ta, void *ptr) {
    if (!ta || !ptr) return 0;

    uint8_t *slot = NULL;
    uint16_t c = 0u;
    if (!locate_user_slot(ta, ptr, &slot, &c)) return 0;

    ta_hdr_t *header = (ta_hdr_t *)slot;
    if (header->class_idx != c || header->flags != TA_FLAG_ALLOCATED) return 0;
    return ta->class_sz[c];
}

size_t tiny_total_free_bytes(const tiny_alloc_t *ta) {
    if (!allocator_layout_valid(ta)) return 0;

    /* Inspect each physical slot once instead of following possibly corrupted
     * free-list links; this cannot loop on a cycle created by memory damage. */
    size_t total = 0u;
    uintptr_t p = (uintptr_t)ta->heap_begin;
    const uintptr_t end = (uintptr_t)ta->heap_end;
    while (p < end) {
        int made_progress = 0;
        for (uint16_t c = 0; c < ta->num_classes; ++c) {
            const size_t slot = ta->slot_sz[c];
            if ((size_t)(end - p) >= slot) {
                const ta_hdr_t *header = (const ta_hdr_t *)p;
                if (header->class_idx == c && header->flags == TA_FLAG_FREE) {
                    total += slot;
                }
                p += slot;
                made_progress = 1;
            }
        }
        if (!made_progress) break;
    }
    return total;
}

size_t tiny_total_slot_bytes(const tiny_alloc_t *ta) {
    if (!allocator_layout_valid(ta)) return 0;
    return (size_t)((uintptr_t)ta->heap_end - (uintptr_t)ta->heap_begin);
}
