/**
 * @file tiny_alloc.h
 * @brief Hardened tiny allocator for embedded targets using a caller-supplied heap.
 *
 * Design:
 *  - Segregated free lists with a few small size classes
 *    (defaults: 16, 32, 64, 96, 128, 192 bytes).
 *  - Caller provides the heap buffer (e.g., static .bss array) and its size.
 *  - No malloc/calloc/free from libc; all state is in tiny_alloc_t (POD).
 *  - No locks/IRQs: single-threaded / single-core critical-path friendly.
 *  - Allocation uses segregated free lists. Allocation and pointer-taking
 *    operations validate slot state and exact boundaries; successful checks
 *    can walk the deterministic carved layout in O(number of slots).
 *
 * Notes:
 *  - Alignment: returns pointers aligned to sizeof(void*).
 *  - Overhead: a 4-byte header plus any padding needed to align the payload.
 *  - Speed > space: the heap is greedily carved into slots at init; no coalescing.
 *  - Invalid pointers, double frees, corrupt slot headers, and invalid class
 *    tables fail closed. Error reporting is intentionally not part of this API.
 */
#ifndef POGO_UTILS_TINY_ALLOC_H
#define POGO_UTILS_TINY_ALLOC_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdint.h>
#include <stddef.h>

/** Maximum number of size classes supported. Keep small for O(1) lookups. */
#ifndef TINY_ALLOC_MAX_CLASSES
#define TINY_ALLOC_MAX_CLASSES 8
#endif

/** Tiny allocator handle (POD). */
typedef struct {
    /* Configuration (immutable after init) */
    uint8_t  num_classes;                 /* number of active size classes */
    uint16_t class_sz[TINY_ALLOC_MAX_CLASSES];  /* payload sizes (bytes), ascending */
    uint16_t slot_sz[TINY_ALLOC_MAX_CLASSES];   /* slot size = header + payload (aligned) */

    /* Free lists, one per class (singly linked via first word in slot). */
    void *free_head[TINY_ALLOC_MAX_CLASSES];

    /* Heap range (for introspection/debug) */
    uint8_t *heap_begin;
    uint8_t *heap_end;
} tiny_alloc_t;

/**
 * @brief Initialize the allocator on a caller-supplied heap.
 *
 * @param ta         Allocator handle (zero-/stack-allocated).
 * @param heap_ptr   Pointer to the heap memory (e.g., static uint8_t buf[4096]).
 * @param heap_bytes Size of the heap in bytes.
 * @param classes    Optional array of positive payload class sizes (bytes),
 *                   strictly ascending. If NULL, defaults to
 *                   {16, 32, 64, 96, 128, 192}.
 * @param num_classes Number of entries in `classes`. If 0, uses default. A
 *                    value above `TINY_ALLOC_MAX_CLASSES` is rejected.
 *
 * The heap is carved in round-robin class order into fixed slots. Unused tail
 * bytes smaller than every slot size are ignored.
 * Invalid arguments, class ordering, or a slot size that cannot be represented
 * leave `ta` cleared and disabled; subsequent allocations return NULL.
 */
void tiny_alloc_init(tiny_alloc_t *ta,
                     void *heap_ptr, size_t heap_bytes,
                     const uint16_t *classes, uint8_t num_classes);

/** Allocate at least `nbytes`. Returns NULL if no slot available. */
void *tiny_malloc(tiny_alloc_t *ta, size_t nbytes);

/** Allocate and zero-initialize. Returns NULL if no slot available. */
void *tiny_calloc(tiny_alloc_t *ta, size_t count, size_t size);

/**
 * Free a live pointer returned by this allocator. NULL, invalid pointers, and
 * repeated frees are ignored after an exact slot-boundary/state check.
 */
void tiny_free(tiny_alloc_t *ta, void *ptr);

/**
 * @brief Reallocate pointer to a new size.
 * Fast-path: if it fits in the same class, return the same pointer.
 * Invalid or already-freed pointers return NULL.
 */
void *tiny_realloc(tiny_alloc_t *ta, void *ptr, size_t new_size);

/* Optional helpers / introspection. tiny_usable_size returns 0 for invalid or
 * freed pointers. Free-byte accounting does not follow free-list links. */
size_t tiny_usable_size(tiny_alloc_t *ta, void *ptr);
size_t tiny_total_free_bytes(const tiny_alloc_t *ta);
size_t tiny_total_slot_bytes(const tiny_alloc_t *ta);

#ifdef __cplusplus
}
#endif
#endif /* POGO_UTILS_TINY_ALLOC_H */
