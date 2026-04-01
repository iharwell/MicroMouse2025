#!/usr/bin/env python3
"""
Universal MMLOG unpacker with sidecar event support.

Supports:
- original typed-schema variant (example format)
- Codex-derived 64-byte-header timing/log variant shown in timing_boot.mmlog
- optional sidecar event files, preferably resolved from the file's own declared sidecar name

Default root: D:\\
Outputs CSV files next to each .mmlog unless --out-dir is provided.
"""
from __future__ import annotations
import argparse
import csv
import struct
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Tuple, Optional, Iterable

MAGIC = b"MMLOG1\x00\x00"

HEADER64 = struct.Struct("<8s14I")          # actual uploaded file layout
HEADER68 = struct.Struct("<8s15I")          # earlier example layout bug/fix family

TEXT_FIELD_HINTS = {
    "stage", "section", "timing_section_id", "event_type", "stream_id", "led_mode",
    "mode", "variant", "tag", "name", "id"
}

KEY_FIELD_CANDIDATES = ["mono_time", "seq", "sequence", "control_tick_index", "t_us"]


@dataclass
class SidecarEvent:
    line_no: int
    original: str
    record_type: str
    ref_raw: str
    ref_num: Optional[int]
    event_kind: str
    message: str
    match_field: str = ""
    match_mode: str = ""
    matched_row_index: Optional[int] = None
    matched_row_key: Optional[int] = None
    delta: Optional[int] = None


def looks_printable_tag4(x: int) -> bool:
    bs = x.to_bytes(4, "little", signed=False)
    return all((32 <= b < 127) or b == 0 for b in bs) and any(b != 0 for b in bs)


def decode_tag4(x: int) -> str:
    bs = x.to_bytes(4, "little", signed=False)
    return ''.join(chr(b) for b in bs if b != 0)


def parse_kv_block(text: str) -> Dict[str, str]:
    out: Dict[str, str] = {}
    for line in text.splitlines():
        line = line.strip()
        if not line or "=" not in line:
            continue
        k, v = line.split("=", 1)
        out[k.strip()] = v.strip()
    return out


def resolve_sidecar_path(log_path: Path, metadata_text: str, notes_text: str) -> Path:
    """Resolve sidecar path using the file's own declared sidecar name first."""
    candidates = []

    for block_text in (notes_text, metadata_text):
        kv = parse_kv_block(block_text)
        for key in ("event_sidecar", "sidecar", "events_sidecar", "event_sidecar_name"):
            value = kv.get(key, "").strip()
            if value:
                candidates.append(value)

    # Fallbacks for older/example variants.
    candidates.append(path_stem_sidecar_name(log_path))

    seen = set()
    for cand in candidates:
        if not cand or cand in seen:
            continue
        seen.add(cand)
        p = Path(cand)
        if p.is_absolute():
            if p.exists():
                return p
        else:
            q = log_path.parent / p
            if q.exists():
                return q
    # Return the authoritative declared path if present, else fallback path even if missing.
    if candidates:
        p = Path(candidates[0])
        return p if p.is_absolute() else (log_path.parent / p)
    return log_path.with_suffix('.events.txt')


def path_stem_sidecar_name(log_path: Path) -> str:
    return f"{log_path.stem}.events.txt"


def detect_header(data: bytes):
    if len(data) < 64 or data[:8] != MAGIC:
        raise ValueError("Not an MMLOG1 file")
    if len(data) >= HEADER64.size:
        vals64 = HEADER64.unpack(data[:HEADER64.size])
        _, version, header_bytes, record_bytes, field_count, metadata_bytes, schema_bytes, notes_bytes, *_ = vals64
        if version > 0 and header_bytes >= HEADER64.size and record_bytes % 4 == 0 and field_count > 0:
            if header_bytes == HEADER64.size + metadata_bytes + schema_bytes + notes_bytes:
                return "header64", vals64, HEADER64.size
    if len(data) >= HEADER68.size:
        vals68 = HEADER68.unpack(data[:HEADER68.size])
        _, version, header_bytes, record_bytes, field_count, metadata_bytes, schema_bytes, notes_bytes, *_ = vals68
        if version > 0 and header_bytes >= HEADER68.size and record_bytes % 4 == 0 and field_count > 0:
            calc = HEADER68.size + metadata_bytes + schema_bytes + notes_bytes
            if header_bytes in (calc, calc - 4):
                return "header68", vals68, HEADER68.size
    vals64 = HEADER64.unpack(data[:HEADER64.size])
    return "header64-best-effort", vals64, HEADER64.size


def parse_schema(schema_text: str) -> List[Tuple[str, str]]:
    raw = [s.strip() for s in schema_text.replace("\n", "").split(",") if s.strip()]
    fields: List[Tuple[str, str]] = []
    for item in raw:
        if ":" in item:
            t, name = item.split(":", 1)
            fields.append((name.strip(), t.strip().lower()))
        else:
            fields.append((item, "u32"))
    return fields


def parse_sidecar(sidecar_path: Path) -> List[SidecarEvent]:
    events: List[SidecarEvent] = []
    if not sidecar_path.exists():
        return events
    with sidecar_path.open("r", encoding="utf-8", errors="replace") as f:
        for line_no, raw_line in enumerate(f, 1):
            line = raw_line.strip()
            if not line:
                continue
            if line.startswith("#"):
                line = line[1:].strip()
            if not line:
                continue
            parts = line.split(",", 3)
            if len(parts) < 4:
                continue
            record_type, ref_raw, event_kind, message = parts[0].strip(), parts[1].strip(), parts[2].strip(), parts[3].strip()
            ref_num: Optional[int]
            try:
                ref_num = int(ref_raw, 0)
            except ValueError:
                ref_num = None
            events.append(SidecarEvent(
                line_no=line_no,
                original=raw_line.rstrip("\n"),
                record_type=record_type,
                ref_raw=ref_raw,
                ref_num=ref_num,
                event_kind=event_kind,
                message=message,
            ))
    return events


def choose_match_field(events: List[SidecarEvent], rows: List[Dict[str, object]]) -> str:
    if not events or not rows:
        return ""
    candidate_names = [name for name in KEY_FIELD_CANDIDATES if name in rows[0]]
    if not candidate_names:
        return ""
    best_name = ""
    best_score = (-1, -1)  # exact_matches, near_matches
    for name in candidate_names:
        try:
            values = [int(row[name]) for row in rows]
        except Exception:
            continue
        value_set = set(values)
        exact = 0
        near = 0
        for ev in events:
            if ev.ref_num is None:
                continue
            if ev.ref_num in value_set:
                exact += 1
            else:
                nearest_delta = min(abs(v - ev.ref_num) for v in values)
                if nearest_delta <= 1000:
                    near += 1
        score = (exact, near)
        if score > best_score:
            best_score = score
            best_name = name
    return best_name


def attach_sidecar(events: List[SidecarEvent], rows: List[Dict[str, object]], key_name: str) -> None:
    if not events or not rows or not key_name:
        return
    values = [int(row[key_name]) for row in rows]
    exact_index = {v: i for i, v in enumerate(values)}
    for ev in events:
        if ev.ref_num is None:
            continue
        if ev.ref_num in exact_index:
            idx = exact_index[ev.ref_num]
            ev.match_field = key_name
            ev.match_mode = "exact"
            ev.matched_row_index = idx
            ev.matched_row_key = values[idx]
            ev.delta = 0
            continue
        # nearest prior within a small tolerance; useful for sidecar events emitted just after a record point
        prior_candidates = [(i, v) for i, v in enumerate(values) if v <= ev.ref_num]
        if prior_candidates:
            idx, val = prior_candidates[-1]
            delta = ev.ref_num - val
            if delta <= 1000:
                ev.match_field = key_name
                ev.match_mode = "nearest_prior"
                ev.matched_row_index = idx
                ev.matched_row_key = val
                ev.delta = delta
                continue
        # absolute nearest within the same tolerance, as a fallback
        idx = min(range(len(values)), key=lambda i: abs(values[i] - ev.ref_num))
        delta = abs(values[idx] - ev.ref_num)
        if delta <= 1000:
            ev.match_field = key_name
            ev.match_mode = "nearest"
            ev.matched_row_index = idx
            ev.matched_row_key = values[idx]
            ev.delta = values[idx] - ev.ref_num


def iter_mmlog_files(root: Path):
    if root.is_file() and root.suffix.lower() == ".mmlog":
        yield root
        return
    for p in root.rglob("*.mmlog"):
        yield p


def unpack_file(path: Path, out_dir: Path) -> List[Path]:
    data = path.read_bytes()
    variant, hdr, physical_header_size = detect_header(data)
    magic = hdr[0]
    version = hdr[1]
    header_bytes = hdr[2]
    record_bytes = hdr[3]
    field_count = hdr[4]
    metadata_bytes = hdr[5]
    schema_bytes = hdr[6]
    notes_bytes = hdr[7]
    flags = hdr[8]
    run_id = hdr[9]
    start_lo = hdr[10]
    start_hi = hdr[11]
    reserved = hdr[12:]

    meta_off = physical_header_size
    schema_off = meta_off + metadata_bytes
    notes_off = schema_off + schema_bytes
    records_off = notes_off + notes_bytes

    metadata_text = data[meta_off:schema_off].decode("utf-8", errors="replace")
    schema_text = data[schema_off:notes_off].decode("utf-8", errors="replace")
    notes_text = data[notes_off:records_off].decode("utf-8", errors="replace")

    fields = parse_schema(schema_text)
    payload = data[records_off:]
    rec_count = len(payload) // record_bytes
    remainder = len(payload) % record_bytes

    base = out_dir / path.stem
    out_dir.mkdir(parents=True, exist_ok=True)
    written: List[Path] = []

    header_csv = base.with_name(base.name + "_header.csv")
    with header_csv.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["key", "value"])
        rows = [
            ("variant", variant),
            ("magic", magic.decode("ascii", errors="replace")),
            ("version", version),
            ("header_bytes_reported", header_bytes),
            ("header_bytes_physical", physical_header_size),
            ("record_bytes", record_bytes),
            ("field_count", field_count),
            ("metadata_bytes", metadata_bytes),
            ("schema_bytes", schema_bytes),
            ("notes_bytes", notes_bytes),
            ("flags", flags),
            ("run_id", run_id),
            ("start_time_us_lo", start_lo),
            ("start_time_us_hi", start_hi),
            ("record_count", rec_count),
            ("payload_remainder_bytes", remainder),
        ]
        for i, rv in enumerate(reserved):
            rows.append((f"reserved_{i}", rv))
        w.writerows(rows)
    written.append(header_csv)

    meta_csv = base.with_name(base.name + "_metadata.csv")
    meta_kv = parse_kv_block(metadata_text)
    with meta_csv.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["key", "value"])
        if meta_kv:
            for k, v in meta_kv.items():
                w.writerow([k, v])
        else:
            w.writerow(["raw", metadata_text])
    written.append(meta_csv)

    schema_csv = base.with_name(base.name + "_schema.csv")
    with schema_csv.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["index", "name", "type"])
        for i, (name, typ) in enumerate(fields):
            w.writerow([i, name, typ])
    written.append(schema_csv)

    notes_csv = base.with_name(base.name + "_notes.csv")
    notes_kv = parse_kv_block(notes_text)
    with notes_csv.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["key", "value"])
        if notes_kv:
            for k, v in notes_kv.items():
                w.writerow([k, v])
        else:
            w.writerow(["raw", notes_text])
    written.append(notes_csv)

    rec_struct = struct.Struct("<" + "I" * len(fields))
    rows: List[Dict[str, object]] = []
    for i in range(rec_count):
        off = records_off + i * record_bytes
        words = rec_struct.unpack(data[off:off + record_bytes])
        row: Dict[str, object] = {}
        for (name, typ), value in zip(fields, words):
            if typ == "f32":
                row[name] = struct.unpack("<f", struct.pack("<I", value))[0]
            elif typ == "i32":
                row[name] = struct.unpack("<i", struct.pack("<I", value))[0]
            else:
                row[name] = value
                lname = name.lower()
                if lname in TEXT_FIELD_HINTS or lname.endswith("_id") or lname.endswith("_type") or lname.endswith("_mode"):
                    if looks_printable_tag4(value):
                        row[f"{name}__decoded"] = decode_tag4(value)
                    else:
                        row[f"{name}__decoded"] = f"0x{value:08X}"
        rows.append(row)

    sidecar_path = resolve_sidecar_path(path, metadata_text, notes_text)
    events = parse_sidecar(sidecar_path)
    match_field = choose_match_field(events, rows)
    attach_sidecar(events, rows, match_field)

    events_by_row: Dict[int, List[SidecarEvent]] = {}
    for ev in events:
        if ev.matched_row_index is not None:
            events_by_row.setdefault(ev.matched_row_index, []).append(ev)

    sidecar_csv = base.with_name(base.name + "_events.csv")
    with sidecar_csv.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow([
            "line_no", "record_type", "ref_raw", "ref_num", "event_kind", "message",
            "match_field", "match_mode", "matched_row_index", "matched_row_key", "delta", "original"
        ])
        for ev in events:
            w.writerow([
                ev.line_no, ev.record_type, ev.ref_raw,
                "" if ev.ref_num is None else ev.ref_num,
                ev.event_kind, ev.message, ev.match_field, ev.match_mode,
                "" if ev.matched_row_index is None else ev.matched_row_index,
                "" if ev.matched_row_key is None else ev.matched_row_key,
                "" if ev.delta is None else ev.delta,
                ev.original,
            ])
    written.append(sidecar_csv)

    data_csv = base.with_name(base.name + "_data.csv")
    field_headers = list(rows[0].keys()) if rows else [name for name, _ in fields]
    extra_headers = [
        "sidecar_event_count", "sidecar_event_kinds", "sidecar_event_messages",
        "sidecar_match_modes", "sidecar_ref_raw"
    ]
    with data_csv.open("w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(field_headers + extra_headers)
        for idx, row in enumerate(rows):
            evs = events_by_row.get(idx, [])
            row_vals = [row.get(h, "") for h in field_headers]
            if evs:
                row_vals += [
                    len(evs),
                    " | ".join(ev.event_kind for ev in evs),
                    " | ".join(ev.message for ev in evs),
                    " | ".join(ev.match_mode for ev in evs),
                    " | ".join(ev.ref_raw for ev in evs),
                ]
            else:
                row_vals += [0, "", "", "", ""]
            w.writerow(row_vals)
    written.append(data_csv)

    return written


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default=r"D:\\", help="File or directory to scan")
    ap.add_argument("--out-dir", default=None, help="Output directory (default: next to each source file)")
    args = ap.parse_args()

    root = Path(args.root)
    files = list(iter_mmlog_files(root))
    if not files:
        print(f"No .mmlog files found under {root}")
        return 1

    for p in files:
        try:
            out = Path(args.out_dir) if args.out_dir else p.parent
            written = unpack_file(p, out)
            print(f"Unpacked {p} ->")
            for w in written:
                print(f"  {w}")
        except Exception as e:
            print(f"Failed to unpack {p}: {e}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
