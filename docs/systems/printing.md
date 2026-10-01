# Lightweight fixed-point printing

[print_format.h](../../src/pogo-utils/print_format.h) supplies a standalone
formatter; [print_log.h](../../src/pogo-utils/print_log.h) queues its output
for [flash_log](flash_logs.md). Neither uses libc printf, floating-point math,
64-bit arithmetic, heap allocation, lookup tables, or automatic background work.
The terminal adapter `printf_fixp()` retains its name/signature but uses the
same small core. Formatting alone does not require `init_fixp()`.

## Syntax and arguments

| Conversion | Argument | Output |
| --- | --- | --- |
| `%d`, `%i` | int | Signed decimal |
| `%u`, `%x`, `%X` | unsigned int | Unsigned decimal/hex |
| `%ld`, `%li`, `%lu`, `%lx`, `%lX` | long / unsigned long | Same, restricted to 32-bit range |
| `%s`, `%.Ns` | String pointer | String, optionally limited to 0..255 bytes; NULL becomes `(null)` |
| `%c`, `%%` | int / none | Character / literal percent |
| `%Q1.15`, `%Q6.10` | Corresponding int16 value, promoted to int | Default 4 / 3 decimal digits |
| `%Q8.24`, `%Q16.16` | Corresponding int32_t value | Default 6 decimal digits |

Numeric width is bounded to 32 characters, with optional zero padding:
`%08x`, `%09.3Q16.16`. `%.NQ...` selects 0..6 decimal digits. Width on strings
or characters, integer precision, other flags/lengths, `*`, pointers, `%n`,
float/double, scientific notation, and 64-bit integer output are unsupported.
There is no fallback. Unknown syntax returns UNSUPPORTED before reading its
argument; streaming output may already contain a prefix. Match argument types
exactly; ordinary printf compiler attributes do not understand these formats.

The fraction is generated digit by digit using 32-bit arithmetic, then rounded
half away from zero with integer carry. Minimum negative values are safe.
Coarse precision may print a rounded endpoint outside the exact representable
range: Q1.15's maximum rounds to `1.0000`; Q8.24's to `128.000000`. Negative
values rounding to zero retain `-0`. This is text rounding, not saturation.

For values currently stored as float, explicitly convert before the variadic
call: `q16_16_from_float(value)`. Its parameter is float and result is integer,
so it avoids float-to-double vararg promotion. This quantizes the value; use
finite inputs and check the chosen format's range/conversion contract.
Do not convert an entire controller to fixed point merely to print diagnostics.

## Bounded buffer and terminal APIs

`pogo_snprintf(out,capacity,&written,format,...)` returns a status and the
actual output length; capacity includes a NUL. Valid nonempty output is
NUL-terminated even on failure. Unlike standard snprintf, it stops on overflow
and does not compute the hypothetical full length. A failed call may leave a
prefix; callers decide whether to publish it. Strings/format must not overlap
the destination. `pogo_vsnprintf` is its va_list counterpart.

`pogo_vformat(write,context,format,args)` emits borrowed spans of at most
32 bytes to a synchronous callback. A false callback return stops formatting.
`printf_fixp(format,...)` uses this with putchar; it is terminal-only and may
block. Its legacy void signature cannot report failures, so unsupported syntax
stops at the prefix. It no longer accepts float or general libc formats;
use explicit standard printf if an application intentionally needs those.
The fixed-point benchmark's float comparisons remain explicit libc calls.

## Flash-first lifecycle

1. During startup, open/initialize each flash_log. Creation/clear may erase
   sectors, and catalog autoformat can lose access to unrelated files.
2. `pogo_log_attach(&printer,&flash,message,sizeof(message))` performs no I/O.
   It defaults to flash-only output; buffer capacity includes a NUL byte.
3. `pogo_log_printf` formats one complete message immediately into that
   caller-supplied storage and appends to the flash RAM cache without I/O.
   Flash-only short records release staging immediately; a spill/mirror
   retains its suffix. Producer strings/stack values are not retained.
4. Regularly call `pogo_log_service(&printer,terminal_byte_budget)`. Each call
   programs at most one full page, retaining any message suffix for later.
5. At an explicit checkpoint, call `pogo_log_flush` across ticks until OK.
   Each call programs at most one page, including partial pages; BUSY means
   more queued/cache/mirror data remains. Partial flush wastes page capacity.

```c
#include "pogo-utils/print_log.h"
#include "pogo-utils/fixp_base.h"

/* Keep all three objects and the buffer in robot USERDATA, not tick locals. */
pogo_flash_log_t flash;
pogo_log_printer_t printer;
char message[96];

/* After successful flash_log initialization, attach once. */
/* pogo_log_attach(&printer, &flash, message, sizeof(message)); */

pogo_print_status_t record_value(uint32_t tick, q16_16_t value) {
    return pogo_log_printf(&printer, "tick=%lu value=%.3Q16.16\n",
                           (unsigned long)tick, value);
}
```

Use separate handles, contexts, and buffers for prints and CSV. The on-flash
format remains the existing append log; use its reader to concatenate payload
bytes and verify each page, not ordinary-file whole-file CRC. The PFFS shell's
`cat` recognizes logs and uses this dedicated reader.

## Backpressure, mirroring, and durability

Multiple flash-only messages fitting the cache can be accepted between service
calls. Only one spill/mirror message is retained per context. BUSY rejects a new message
without modifying the old one. Oversized/unsupported messages never publish
their formatted prefix. FLASH_FULL rejects a complete message that cannot fit
the remaining file/cache payload. `rejected_messages` is a saturating count
of producer rejections; `flash_status` exposes underlying flash failures.
Retry BUSY only if required; otherwise count/drop or defer production. Other
errors need an application policy, not endless retry or recursive logging.

`pogo_log_set_terminal` optionally attaches a transport callback while idle.
The service call invokes it at most once, with at most its requested byte
budget. The callback returns accepted bytes; zero retains pending data, and
a shorter count retains the suffix. Slow/stalled mirroring backpressures this
one-message queue. A zero service budget defers mirroring, not flash work.
No UART/stdout code is linked by this optional callback interface. The example
compiles its putchar adapter only when `PRINT_LOG_TERMINAL=1`.

Accepted in RAM does not mean durable. Even `pogo_log_pending()==false` can
leave bytes in the flash page cache. Reset loses uncommitted data; a message
spanning pages can leave a committed prefix. Flash writes/readback and terminal
callbacks are synchronous: one page / N bytes are work budgets, not timing
guarantees. Calls sharing flash must be serialized, with no reentrant/ISR use.
Do not append/reopen/clear the borrowed handle while pending; finish or
explicitly discard/re-attach the adapter before changing the raw log state.

Memory is one existing 256-byte flash cache, caller-chosen message storage,
and adapter metadata per file. Core scratch is small and bounded, but flash
verification also uses its existing 256-byte stack buffer. Actual firmware,
stack and CPU savings must be measured on Pogobot; other remaining printf
references can keep the libc formatter linked. See the
[dedicated example](../../examples/print_log/README.md).
