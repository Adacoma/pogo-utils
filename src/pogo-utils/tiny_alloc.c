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
    uint16_t flags;     /* reserved (0) */
} ta_hdr_t;

/* Alignment helpers */
static inline size_t ta_align(void) { return sizeof(void*); }
static inline size_t round_up_sz(size_t x, size_t a) {
    size_t r = x % a; return r ? (x + (a - r)) : x;
}
/* Distance from slot base to user payload (and to free-list "next" when FREE) */
static inline size_t hdr_off(void) {
    size_t a = sizeof(void*);
    size_t s = sizeof(ta_hdr_t);
    return (s + (a - 1)) & ~(a - 1); /* ceil to pointer alignment */
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

static inline uint8_t *slot_from_user(void *user_ptr) {
    return (uint8_t *)user_ptr - hdr_off();
}

/* Free-list "next" lives at payload start when the slot is FREE */
static inline void **slot_next_ptr(uint8_t *slot_base) {
    return (void **)(slot_base + hdr_off());
}

static inline void push_free(void **head, uint8_t *slot_base) {
    set_next(slot_base, *head);
    *head = slot_base;
}

static inline void *pop_free(void **head) {
    void *slot = *head;
    if (!slot) return NULL;
    *head = get_next((uint8_t *)slot);
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
    if (!ta || !heap_ptr || heap_bytes < 32) return;

    memset(ta, 0, sizeof(*ta));

    /* defaults covering 16..192 */
    static const uint16_t k_default[] = { 16, 32, 64, 96, 128, 192 };
    if (!classes || !num_classes) {
        classes = k_default;
        num_classes = (uint8_t)(sizeof(k_default)/sizeof(k_default[0]));
    }
    if (num_classes > TINY_ALLOC_MAX_CLASSES) num_classes = TINY_ALLOC_MAX_CLASSES;
    ta->num_classes = num_classes;

    /* pointer-align heap start */
    uint8_t *h = (uint8_t*)heap_ptr;
    uintptr_t ha = round_up_sz((uintptr_t)h, ta_align());
    size_t drop = (size_t)(ha - (uintptr_t)h);
    if (heap_bytes <= drop) return;
    h += drop; heap_bytes -= drop;
    ta->heap_begin = h;

    /* compute slot sizes (aligned) */
    for (int i = 0; i < num_classes; ++i) {
        ta->class_sz[i] = classes[i];
        /* Slot needs: header + max(sizeof(void*) for free-list, payload for user) */
        size_t payload = (size_t)classes[i];
        if (payload < sizeof(void*)) payload = sizeof(void*);  /* need room for free-list ptr */
        size_t slot = round_up_sz(hdr_off() + payload, ta_align());
        if (slot > 0xFFFFu) slot = 0xFFFFu;
        ta->slot_sz[i] = (uint16_t)slot;
    }

    /* carve fairly across classes (round-robin) so each class gets slots */
    uint8_t *p = h, *end = h + heap_bytes;

    /* find the smallest slot size; if we can't fit this, we're done */
    size_t min_slot = ta->slot_sz[0];
    for (uint8_t i = 1; i < num_classes; ++i)
        if (ta->slot_sz[i] < min_slot) min_slot = ta->slot_sz[i];

    while ((size_t)(end - p) >= min_slot) {
        int made_progress = 0;
        for (uint8_t cls = 0; cls < num_classes; ++cls) {
            size_t slot = ta->slot_sz[cls];
            if ((size_t)(end - p) >= slot) {
                ta_hdr_t *hdrp = (ta_hdr_t*)p;
                hdrp->class_idx = cls;
                hdrp->flags = 0;
                push_free(&ta->free_head[cls], p);
                p += slot;
                made_progress = 1;
            }
        }
        if (!made_progress) break; /* leftover < any slot size */
    }
    ta->heap_end = p;
}

void *tiny_malloc(tiny_alloc_t *ta, size_t nbytes) {
    if (!ta || nbytes == 0) return NULL;
    int cls = choose_class(ta, nbytes);
    if (cls < 0) return NULL;
    for (int c = cls; c < ta->num_classes; ++c) {
        uint8_t *slot = pop_free(&ta->free_head[c]);
        if (slot) {
            /* header is at slot, payload starts at slot+hdr_off() */
            ta_hdr_t *h = (ta_hdr_t *)slot;
            h->class_idx = (uint16_t)c;
            h->flags = 0;
            return user_from_slot(slot);
        }
    }
    return NULL;
}

void *tiny_calloc(tiny_alloc_t *ta, size_t count, size_t size) {
    if (!ta) return NULL;
    if (!count || !size) return NULL;
    size_t need = count * size; /* acceptable for small heaps */
    void *p = tiny_malloc(ta, need);
    if (p) memset(p, 0, need);
    return p;
}

static inline uint8_t *ptr_to_slot(void *ptr) { return (uint8_t*)ptr - hdr_off(); }

void tiny_free(tiny_alloc_t *ta, void *ptr) {
    if (!ta || !ptr) return;
    uint8_t *slot = slot_from_user(ptr);
    
    /* Validate slot is within heap bounds */
    if (slot < ta->heap_begin || slot >= ta->heap_end) return;
    
    ta_hdr_t *h = (ta_hdr_t *)slot;
    uint16_t c = h->class_idx;
    if (c >= ta->num_classes) return; /* ignore corrupted block in release builds */
    
    push_free(&ta->free_head[c], slot);
}

void *tiny_realloc(tiny_alloc_t *ta, void *ptr, size_t new_size) {
    if (!ta) return NULL;
    if (!ptr) return tiny_malloc(ta, new_size);
    if (new_size == 0) { tiny_free(ta, ptr); return NULL; }

    uint8_t *slot = slot_from_user(ptr);
    
    /* Validate slot is within heap bounds */
    if (slot < ta->heap_begin || slot >= ta->heap_end) return NULL;
    
    ta_hdr_t *h = (ta_hdr_t *)slot;
    int old_c = h->class_idx;
    if (old_c >= ta->num_classes) return NULL;  /* bounds check! */
    size_t old_payload = ta->class_sz[old_c];

    int new_c = choose_class(ta, new_size);
    if (new_c < 0) return NULL;

    /* Fast path: fits in the same class */
    if (new_c == old_c) return ptr;

    /* Allocate new, copy, free old */
    void *np = tiny_malloc(ta, new_size);
    if (!np) return NULL;
    size_t new_payload = ta->class_sz[new_c];  /* FIX: use class_sz, not slot_sz */
    size_t n = old_payload < new_payload ? old_payload : new_payload;
    memcpy(np, ptr, n);
    tiny_free(ta, ptr);
    return np;
}

size_t tiny_usable_size(tiny_alloc_t *ta, void *ptr) {
    if (!ta || !ptr) return 0;
    uint8_t *slot = ptr_to_slot(ptr);
    
    /* Validate slot is within heap bounds */
    if (slot < ta->heap_begin || slot >= ta->heap_end) return 0;
    
    ta_hdr_t *hdrp = (ta_hdr_t*)slot;
    if (hdrp->class_idx >= ta->num_classes) return 0;
    return ta->class_sz[hdrp->class_idx];
}

size_t tiny_total_free_bytes(const tiny_alloc_t *ta) {
    if (!ta) return 0;
    size_t s = 0;
    for (int c = 0; c < ta->num_classes; ++c) {
        size_t slot = ta->slot_sz[c];
        for (void *p = ta->free_head[c]; p; p = get_next((uint8_t*)p)) s += slot;
    }
    return s;
}

size_t tiny_total_slot_bytes(const tiny_alloc_t *ta) {
    if (!ta) return 0;
    return (size_t)(ta->heap_end - ta->heap_begin);
}

