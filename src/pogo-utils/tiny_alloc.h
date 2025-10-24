/**
 * @file tiny_alloc.h
 * @brief O(1)-ish tiny allocator for embedded targets using a caller-supplied heap.
 *
 * Design:
 *  - Segregated free lists with a few small size classes (defaults: 16, 32, 64, 96, 128 bytes).
 *  - Caller provides the heap buffer (e.g., static .bss array) and its size.
 *  - No malloc/calloc/free from libc; all state is in tiny_alloc_t (POD).
 *  - No locks/IRQs: single-threaded / single-core critical-path friendly.
 *  - ~O(1) alloc/free: pop/push a singly linked list for the chosen size class.
 *
 * Notes:
 *  - Alignment: returns pointers aligned to sizeof(void*).
 *  - Overhead: 4 bytes per allocation for a tiny header (class index).
 *  - Speed > space: the heap is greedily carved into slots at init; no coalescing.
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
 * @param classes    Optional array of payload class sizes (bytes), ascending.
 *                   If NULL, defaults to {16, 32, 64, 96, 128}.
 * @param num_classes Number of entries in `classes`. If 0, uses default.
 *
 * The heap is carved greedily from smallest to largest class into fixed slots.
 * Unused tail bytes (< smallest slot size) are ignored.
 */
void tiny_alloc_init(tiny_alloc_t *ta,
                     void *heap_ptr, size_t heap_bytes,
                     const uint16_t *classes, uint8_t num_classes);

/** Allocate at least `nbytes`. Returns NULL if no slot available. */
void *tiny_malloc(tiny_alloc_t *ta, size_t nbytes);

/** Allocate and zero-initialize. Returns NULL if no slot available. */
void *tiny_calloc(tiny_alloc_t *ta, size_t count, size_t size);

/** Free a pointer obtained from tiny_malloc/tiny_calloc. No-op if NULL. */
void tiny_free(tiny_alloc_t *ta, void *ptr);

/**
 * @brief Reallocate pointer to a new size.
 * Fast-path: if it fits in the same class, return the same pointer.
 */
void *tiny_realloc(tiny_alloc_t *ta, void *ptr, size_t new_size);

/* Optional helpers / introspection */
size_t tiny_usable_size(tiny_alloc_t *ta, void *ptr);
size_t tiny_total_free_bytes(const tiny_alloc_t *ta);
size_t tiny_total_slot_bytes(const tiny_alloc_t *ta);

#ifdef __cplusplus
}
#endif
#endif /* POGO_UTILS_TINY_ALLOC_H */

