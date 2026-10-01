# tiny_alloc

[tiny_alloc.h](../../src/pogo-utils/tiny_alloc.h) is a bounded allocator over
a caller-supplied arena, without libc malloc, coalescing, locks, or interrupts.
It suits small fixed-size scratch objects; it is not a general heap replacement.

## Layout and sizing

Call `tiny_alloc_init(&ta,arena,bytes,classes,count)`.
NULL classes/zero count use payload sizes 16, 32, 64, 96, 128, and 192 bytes.
Custom sizes must be positive, strictly ascending, and representable with
header/alignment overhead; there are at most `TINY_ALLOC_MAX_CLASSES`
(default 8). Invalid initialization clears/disables the handle.

The arena is aligned and carved in round-robin class order, with a four-byte
slot header plus payload alignment to sizeof(void*). Tail bytes too small for a
slot are ignored. Each class owns a free list. Allocation starts with the
smallest suitable class and can use a larger free class; it does not split a
large slot or merge neighbors.

Total free payload is not a guarantee that a requested block fits.
Estimate individual request sizes, counts and simultaneous lifetimes. Pointer
alignment is not a blanket over-alignment guarantee for SIMD or unusual types.
The opaque optimizer handle alone can exceed default largest classes.

## Operations and failure behavior

`tiny_malloc` returns a slot or NULL. `tiny_calloc` checks multiplication
overflow then clears the requested bytes. `tiny_usable_size` reports the live
slot's payload capacity. `tiny_free` ignores NULL, invalid/non-boundary pointers,
corrupt headers, and repeated frees after validation. `tiny_realloc` retains
a fitting slot or allocates/copies/frees; check failure before replacing the
caller's pointer. Inspect zero-size behavior in the implementation if it is
part of your API design, rather than treating this as exact libc semantics.

Size/address arithmetic, ordering, exact boundaries, and allocation state are
validated. Some successful validation paths walk the deterministically carved
layout in O(number of slots), so the hardened implementation is not uniformly
O(1). Silent rejection avoids growing an error-reporting API but complicates
diagnostics; add application-level allocation failure counters when needed.

The allocator cannot prevent writes past a live payload, use-after-free in
application code, stale-pointer reuse after a new allocation takes the same
slot, or arbitrary corruption of user memory. Its double-free protection is
API-state validation, not a garbage collector or memory sanitizer.

## Ownership and integration

Keep both `tiny_alloc_t` and the arena alive until every allocation is released.
Do not reinitialize the arena with live pointers, mix allocators, or share a
handle concurrently. In Pogosim place mutable arenas in per-robot USERDATA.
Statically reserving an arena costs RAM even while no slots are allocated.

`tiny_total_free_bytes` and `tiny_total_slot_bytes` support diagnostics;
free accounting avoids following potentially corrupt free-list links.
They do not measure internal fragmentation, maximum allocation availability,
or peak stack use.

See [tiny_alloc example](../../examples/tiny_alloc/README.md),
[optimization guide](optimization.md), and
[extension guide](../extending.md). Use caller-owned static workspaces instead
when maximum dimensions are known and dynamic lifetimes add no value.
