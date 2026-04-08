# Micromouse Logging File Format Specification (RevH)

## 1. Purpose

This specification defines a compact, row-oriented logging format intended to replace `.csv` for micromouse timing, measurement, and other structured tabular logs that must be written entirely inside the control loop.

This format is for **tabular binary data**. It is not intended to replace free-form console logging. Console-style logs should use a separate text file such as `log.txt`.

The design goal is to preserve the key benefits of `.csv`:

- ordered rows,
- labeled columns,
- known field types,
- simple offline decoding,
- optional categorical/string labeling,

while avoiding the runtime cost of formatting text for every control-loop iteration.

---

## 2. Division of responsibility

This file-format specification defines:

- the primary `.mmlog` file structure,
- the required sidecar reference mechanism,
- the sidecar schema format,
- field-type encoding rules,
- record ordering rules,
- the required metadata core,
- and the optional label-table section.

Application specifications define:

- which rows are emitted,
- when rows are emitted,
- what the fields mean,
- field units and semantics,
- which fields are allowed to use label hashing,
- and mode-specific acceptance criteria.

The file-format specification defines **how rows are laid out and labeled**.

The application specification defines **what those rows mean**.

---

## 3. Shared operating rule

### 3.1 Control-loop-bounded logging

All modes using this format shall assume that logging work is performed inside the control loop.

Append cost shall therefore remain bounded and suitable for real-time use.

### 3.2 One-primary-row-per-loop rule

Under normal operation, a stream using this format shall emit **at most one primary row per control-loop iteration**, regardless of mode.

### 3.3 Fault exception

A fault may emit one additional row only if the fault halts or aborts further normal control-loop progress.

### 3.4 Tabular rule

The primary parser model is that each primary record corresponds to one row in a logical table, in the same sense that each line of a `.csv` file corresponds to one row.

---

## 4. Primary `.mmlog` file structure

### 4.1 Required file opening line

Every primary `.mmlog` file shall begin with exactly one ASCII text line of the form:

```text
sidecar_file=xxxxxxxx
```

where `xxxxxxxx` is the file name of the bound sidecar file.

Example:

```text
sidecar_file=noise_main.sidecar
```

### 4.2 Placement rule

This `sidecar_file=...` line shall be the **only item before binary data begins**.

No additional text header, magic number, schema block, comment block, JSON block, or metadata block shall appear in the primary `.mmlog` file before the first data row.

Anything beyond the sidecar binding line shall be placed in the sidecar file.

### 4.3 Line termination

The `sidecar_file=...` line shall terminate with a single newline character.

Recommended default:

- `\n` (LF)

Parsers may tolerate `\r\n`, but writers should emit `\n`.

### 4.4 Binary-data start

The first byte immediately following the terminating newline of the `sidecar_file=...` line is the first byte of row data.

---

## 5. Row model

### 5.1 Single row schema per file

Each `.mmlog` file shall represent exactly one primary tabular row schema.

The row layout is defined by the ordered header entries in the sidecar file.

The structure may be chosen arbitrarily at design time, but once a run begins the row structure for that file is locked and shall not change.

### 5.2 Ordered-field rule

Field order in each binary row shall match the order of the header entries in the sidecar file exactly.

This is intentionally analogous to `.csv`, where column order is determined by header order.

### 5.3 Fixed row width

Primary rows shall be fixed width.

One `.mmlog` file corresponds to one fixed-width primary row layout for the duration of the run.

### 5.4 No mixed row layouts within one file

If an application needs a different row structure, it shall use a different `.mmlog` file with its own sidecar.

Mixed row layouts, mixed schemas, and per-row structural changes are forbidden within one `.mmlog` file.

This format is intentionally locked down at run time so that appending rows and decoding rows remain simple and deterministic.

### 5.4 `row_bytes` requirement

The sidecar file shall include a required metadata line:

```text
row_bytes=NN
```

where `NN` is the total row width in bytes.

This value shall equal the sum of the widths implied by the ordered header entries.

The firmware writer shall serialize rows in that exact width with no undocumented padding.

The host parser shall verify that:

- the parsed header width equals `row_bytes`, and
- the binary payload length is an integer multiple of `row_bytes`.

---

## 6. Scalar field types

### 6.1 Allowed baseline types

The baseline allowed scalar types are:

- `u8`
- `i8`
- `u16`
- `i16`
- `u32`
- `i32`
- `f32`
- `s8`
- `s16`
- `s32`

These are the type prefixes used in the sidecar header line.

### 6.2 Encoding rules

All multi-byte scalar fields shall be stored in **little-endian** byte order by default.

A sidecar may override this default to **big-endian** only if the **second line** of the sidecar file is exactly:

```text
byte_order=BE
```

If this line is absent, writers shall emit little-endian multi-byte fields and parsers shall interpret multi-byte fields as little-endian.

No other placement of `byte_order=BE` is valid. In particular, a `byte_order=BE` line appearing later in the sidecar shall be rejected.

Field encodings:

- `u8`  = unsigned 8-bit integer
- `i8`  = signed 8-bit integer
- `u16` = unsigned 16-bit integer, using the file byte order
- `i16` = signed 16-bit integer, using the file byte order
- `u32` = unsigned 32-bit integer, using the file byte order
- `i32` = signed 32-bit integer, using the file byte order
- `f32` = IEEE-754 single-precision floating point, using the file byte order
- `s8`  = 8-bit string-hash field
- `s16` = 16-bit string-hash field, using the file byte order
- `s32` = 32-bit string-hash field, using the file byte order

### 6.3 Meaning of `s8`, `s16`, and `s32`

A field with prefix `s8`, `s16`, or `s32` is a **string-reference hash field**.

It does not store the string bytes directly in the row.

Instead, the row stores the hash of a label string. The host may then match that hash against the strings listed in the sidecar label table.

Example header entries:

- `s32_section`
- `s32_event`
- `s16_primitive`

This mechanism allows an arbitrary string to be logged without placing variable-length text in the binary row.

### 6.4 Fixed hash algorithm for string-reference fields

All `s8`, `s16`, and `s32` fields shall use **FNV-1a** as the hash algorithm.

The hash input shall be the exact UTF-8 byte sequence of the label string as written in the sidecar label table, excluding the line terminator.

Hash-width rules:

- `s32` stores the full 32-bit FNV-1a hash
- `s16` stores the low 16 bits of the 32-bit FNV-1a hash
- `s8` stores the low 8 bits of the 32-bit FNV-1a hash

Writers and host-side tools shall use the same rule.

No sidecar metadata entry is required to select or override the hash algorithm. The hash algorithm is fixed by this format revision.

### 6.5 Missing-value convention

Missing-value rules are application-defined, but they shall be documented in the sidecar metadata if used.

Recommended defaults:

- integer sentinels: all-ones value of the field width
- `f32`: quiet NaN
- `s8`, `s16`, `s32`: all-ones value of the field width reserved as the missing sentinel unless the application defines otherwise

The format allows other choices, but they must be documented in the sidecar file if they differ from the recommended defaults.

---

## 7. Sidecar file purpose

The sidecar file is the authoritative metadata source for the `.mmlog` file.

It shall provide the information needed to decode the binary rows into a labeled table.

At minimum, the sidecar shall carry:

- the required metadata core,
- the ordered header list,
- the field types via prefixes on the field names,
- and any additional metadata the application wants to bind to the file.

The sidecar replaces the role that the header row serves in `.csv`, and it may also carry additional file-level metadata.

---

## 8. Sidecar file structure

### 8.1 Overview

The sidecar file is a UTF-8 text file.

It has three logical parts, in this order:

1. required and optional metadata lines,
2. one mandatory header line,
3. optional label-table section.

### 8.2 Required metadata core

The sidecar shall include at least these metadata lines before the header line:

```text
schema_version=NN
row_bytes=NN
```

An optional byte-order declaration may appear only as the **second line** of the sidecar:

```text
byte_order=BE
```

If this line is absent, the file uses little-endian multi-byte storage by default.

Additional metadata lines may also be included in `key=value` form.

Examples:

Little-endian by default:

```text
schema_version=2
row_bytes=40
row_kind=primary
units_time=us
missing_f32=nan
```

Big-endian override:

```text
schema_version=2
byte_order=BE
row_bytes=40
row_kind=primary
units_time=us
missing_f32=nan
```

### 8.3 Metadata-line rules

Metadata lines shall use the form:

```text
key=value
```

The first non-`key=value` line is the mandatory header line.

Recommended parser rules:

- duplicate metadata keys should be rejected,
- keys are case-sensitive,
- values are taken literally after the first `=`,
- blank lines should be rejected unless an application specification explicitly allows them.

Additional byte-order rule:

- `byte_order=BE`, if present, shall appear only as the second line of the sidecar,
- if the second line is not `byte_order=BE`, the file shall be interpreted as little-endian,
- parsers should reject any later `byte_order` metadata key as malformed for this format revision.

### 8.4 Mandatory header line

The sidecar shall contain exactly one mandatory header line that defines the row layout.

This header shall be a simple comma-separated line, analogous to a `.csv` header row.

Each header entry shall be of the form:

```text
type_fieldname
```

where `type` is one of the allowed field-type prefixes from Section 6.

Example:

```text
i32_time,u16_dt_tick_us,f32_innovation_rms,s32_section,u8_flags
```

Another example:

```text
u32_seq,u32_t_tick_start_us,u16_dt_tick_us,u16_dt_body_us,u16_t_imu_ready_us,s32_stage,u32_flags
```

The order of entries in this line defines the field order in each binary row.

### 8.5 Header naming rule

The first underscore separates the type prefix from the field name.

Examples:

- `i32_time`
- `u16_dt_tick_us`
- `f32_encoder_rate_l`
- `s32_event`

The field name portion may itself contain additional underscores.

### 8.6 Label-table section

If the file uses a label table, the sidecar shall contain a line consisting of exactly:

```text
LABELS:
```

This line shall appear after the mandatory header line.

All further lines in the sidecar file shall then be entries in the label set.

Each line after `LABELS:` is one label entry.

The first label line has index `0`, the second has index `1`, and so on.

Example:

```text
schema_version=2
row_bytes=20
u32_seq,s32_section,u32_flags,f32_value
LABELS:
SEC_00_TIMING
SEC_10_STATIC
SEC_20_LAUNCH
FAULT_STORAGE
```

### 8.7 No content after label table begins

Once the line `LABELS:` appears, all remaining lines belong to the label set.

No further metadata lines or additional schema lines shall appear after `LABELS:`.

### 8.8 Append-friendly label-table rule

Because the label table is required to appear at the end of the sidecar file, firmware may append new label strings to the label-table section as needed during logging.

This allows the firmware to:

1. hash a string into an `s8`, `s16`, or `s32` field,
2. append the original string to the label-table section if not already present,
3. continue writing fixed-width binary rows without placing variable-length text in the main `.mmlog` file.

The application specification shall define whether duplicate label lines are allowed or whether the firmware must de-duplicate labels before appending.

---

## 9. Label-table usage

### 9.1 Purpose

The label table exists so that rows may refer to strings compactly without storing text in the binary stream.

Typical uses include:

- stage names,
- section names,
- primitive names,
- event names,
- fault names,
- or other categorical values.

### 9.2 Reference methods

There are two supported reference methods.

#### Method A: integer index reference

A row may refer to a label entry by storing its numeric index in an integer field.

The meaning of that field is application-defined.

Example:

- sidecar label entry `5` = `SEC_50_SMOOTH`
- binary row stores `u16_section_id = 5`

#### Method B: string-hash reference

A row may refer to a label entry by storing the hash of the label string in an `s8`, `s16`, or `s32` field.

The host shall compute the configured hash over each label-table entry and match the row value against the label hashes.

Example:

- sidecar label entry `SEC_50_SMOOTH`
- host computes the 32-bit FNV-1a hash of `"SEC_50_SMOOTH"`
- binary row stores that 32-bit value in `s32_section`

This method allows an arbitrary string to be logged if needed, provided the string appears in the label table.

### 9.3 Collision handling

Hash collisions are possible for `s8`, `s16`, and `s32`.

The application specification shall define the required behavior if collisions occur.

Recommended rule:

- host tooling shall detect collisions within the active label set,
- if a collision exists for a field that is expected to decode uniquely, the decode shall be flagged as ambiguous,
- and the application should move to a wider hash field or tighter label set if ambiguity is unacceptable.

Because `s8` and `s16` are truncated forms of the 32-bit FNV-1a hash, they are more collision-prone than `s32`. `s32` should be preferred unless row-width pressure is material.

### 9.4 Optionality

A label table is optional.

If a file does not need string-table references, the sidecar may end after the header line.

---

## 10. Row-size calculation

### 10.1 Rule

Row size is computed as the sum of the field widths implied by the ordered header entries.

Widths are:

- `u8`, `i8`, `s8`     = 1 byte
- `u16`, `i16`, `s16`  = 2 bytes
- `u32`, `i32`, `f32`, `s32` = 4 bytes

### 10.2 Example

Header:

```text
u32_seq,u16_dt_tick_us,u16_t_imu_ready_us,f32_innovation_rms,s32_section,u8_flags
```

Row width:

- `u32_seq` = 4
- `u16_dt_tick_us` = 2
- `u16_t_imu_ready_us` = 2
- `f32_innovation_rms` = 4
- `s32_section` = 4
- `u8_flags` = 1

Total = **17 bytes per row**

The sidecar for this header shall therefore contain:

```text
row_bytes=17
```

### 10.3 Firmware requirement

Firmware shall serialize rows in that exact packed order with no undocumented padding.

This may be implemented by:

- a packed structure with compile-time size enforcement, or
- manual field-by-field serialization.

---

## 11. Parser expectations

A parser for this format shall perform the following steps:

1. Open the `.mmlog` file.
2. Read the first line.
3. Parse `sidecar_file=...`.
4. Open the referenced sidecar file.
5. Read the first sidecar line and verify that it provides `schema_version`.
6. Read the second sidecar line.
7. If the second line is exactly `byte_order=BE`, set the multi-byte field byte order to big-endian and continue reading metadata lines.
8. Otherwise, set the multi-byte field byte order to little-endian by default and treat that second line as the next ordinary sidecar line.
9. Continue reading metadata lines until the first non-`key=value` line.
10. Verify that `schema_version` and `row_bytes` are present.
11. Reject the file if any `byte_order` metadata line appears anywhere other than the second sidecar line.
12. Interpret the first non-`key=value` line as the mandatory header line.
13. Parse the ordered `type_fieldname` entries.
14. Compute the row width from the header and verify that it matches `row_bytes`.
15. If a line `LABELS:` appears later, treat all following lines as label entries.
16. Decode the remaining bytes of the `.mmlog` file as repeated rows using the ordered field layout and the resolved byte order.
17. Verify that the binary payload size is an integer multiple of `row_bytes`.
18. If `s8`, `s16`, or `s32` fields are present and a label table exists, optionally hash the label strings host-side to enable string matching.
19. Reject the file if decoding would require more than one row schema within the same `.mmlog` file.

Parsers shall not expect any additional binary or textual header in the `.mmlog` file beyond the `sidecar_file=...` line.

---

## 12. Recommended file naming

Recommended naming pattern:

- primary data file: `name.mmlog`
- sidecar file: `name.sidecar`

Examples:

- `noise_main.mmlog`
- `noise_main.sidecar`

- `timing_tick.mmlog`
- `timing_tick.sidecar`

The sidecar file name written in the `.mmlog` file may be a simple file name or a relative path, as defined by the application.

---

## 13. Minimal example

### 13.1 Primary file beginning

```text
sidecar_file=timing_tick.sidecar
```

Binary row data begins immediately after the newline.

### 13.2 Sidecar file

Little-endian default example:

```text
schema_version=2
row_bytes=20
u32_seq,u16_dt_tick_us,f32_innovation_rms,s32_stage,u32_flags
LABELS:
STAGE_TIMING
STAGE_STATIC
STAGE_LAUNCH
STAGE_STRAIGHT
STAGE_YAW
STAGE_SMOOTH
STAGE_LOOP_CW
STAGE_LOOP_CCW
```

Big-endian override example:

```text
schema_version=2
byte_order=BE
row_bytes=20
u32_seq,u16_dt_tick_us,f32_innovation_rms,s32_stage,u32_flags
LABELS:
STAGE_TIMING
STAGE_STATIC
STAGE_LAUNCH
STAGE_STRAIGHT
STAGE_YAW
STAGE_SMOOTH
STAGE_LOOP_CW
STAGE_LOOP_CCW
```

---

## 14. Acceptance criteria

A file conforms to this specification only if all of the following are true:

1. The primary `.mmlog` file begins with exactly one `sidecar_file=...` line.
2. No other item appears before binary row data begins.
3. The referenced sidecar file exists and is readable.
4. The sidecar file contains `schema_version` and `row_bytes` metadata lines before the header line.
5. If `byte_order=BE` is present, it appears exactly as the second line of the sidecar.
6. If the second sidecar line is not `byte_order=BE`, multi-byte scalars are little-endian by default.
7. The sidecar file contains exactly one mandatory header line.
8. The header line is a simple comma-separated line of `type_fieldname` entries.
9. Row bytes are ordered exactly as implied by the header order.
10. The computed header width equals `row_bytes`.
11. The binary payload length is an integer multiple of `row_bytes`.
12. All multi-byte scalars use the byte order selected by the sidecar rule above.
13. If `LABELS:` appears, all following lines are label entries.
14. The file remains decodable into a labeled table without requiring any hidden schema outside the sidecar.
15. All rows in a given `.mmlog` file use the same fixed-width schema for the full run.
16. If a different row structure is needed, it is written to a different file with a different sidecar.

---

## 15. Rationale

This design is intentionally close to `.csv` at the schema level:

- `.csv` stores rows in order,
- `.csv` uses a header row to define column order and names,
- this format stores compact binary rows in order,
- and the sidecar file provides the header row plus type information and optional additional metadata.

The label-table extension adds a control-loop-safe way to refer to strings:

- the row stores a fixed-width value,
- the sidecar stores the human-readable strings,
- and the host may reconstruct the mapping by index or by hash.

The `.mmlog` file therefore carries only:

- a sidecar binding line,
- and then the row data.

Everything else that would otherwise complicate the hot-path file writer is pushed into the sidecar.

A different row structure therefore means a different file, not a different row type inside the same file.
