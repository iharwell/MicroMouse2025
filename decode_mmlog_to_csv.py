
#!/usr/bin/env python3
r"""
Decode Micromouse .mmlog files into .csv files.

Default behavior:
- Recursively scans D:\ for .mmlog files.
- Reads the sidecar referenced by the first line of each .mmlog.
- Verifies schema_version / row_bytes and row-width consistency.
- Decodes the fixed-width binary rows into CSV.
- Writes one CSV next to each .mmlog file using the same stem.

For s8/s16/s32 fields:
- The CSV contains both the raw stored value (<field>__raw) and the decoded label (<field>).
- Label decoding uses FNV-1a, per RevG of the format spec.
"""

from __future__ import annotations

import argparse
import csv
import struct
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, Iterable, List, Sequence, Tuple


TYPE_INFO: Dict[str, Tuple[str, int]] = {
    "u8": ("B", 1),
    "i8": ("b", 1),
    "u16": ("H", 2),
    "i16": ("h", 2),
    "u32": ("I", 4),
    "i32": ("i", 4),
    "f32": ("f", 4),
    "s8": ("B", 1),
    "s16": ("H", 2),
    "s32": ("I", 4),
}

FNV1A_32_OFFSET = 0x811C9DC5
FNV1A_32_PRIME = 0x01000193


class MMLogError(Exception):
    """Raised when a file does not conform to the expected .mmlog format."""


@dataclass(frozen=True)
class FieldSpec:
    type_name: str
    field_name: str
    struct_code: str
    width: int

    @property
    def is_string_hash(self) -> bool:
        return self.type_name in {"s8", "s16", "s32"}

    @property
    def hash_bits(self) -> int:
        if self.type_name == "s8":
            return 8
        if self.type_name == "s16":
            return 16
        if self.type_name == "s32":
            return 32
        raise ValueError(f"{self.type_name} is not a string-hash type")


@dataclass
class SidecarSpec:
    metadata: Dict[str, str]
    fields: List[FieldSpec]
    labels: List[str]
    row_bytes: int

    @property
    def struct_format(self) -> str:
        return "<" + "".join(field.struct_code for field in self.fields)


def fnv1a_32(data: bytes) -> int:
    h = FNV1A_32_OFFSET
    for b in data:
        h ^= b
        h = (h * FNV1A_32_PRIME) & 0xFFFFFFFF
    return h


def truncate_hash(value: int, bits: int) -> int:
    mask = (1 << bits) - 1
    return value & mask


def normalize_string_hash_metadata(value: str) -> str:
    return value.strip().lower().replace("-", "").replace("_", "")


def parse_header_entry(entry: str) -> FieldSpec:
    if "_" not in entry:
        raise MMLogError(f"Invalid header entry without type prefix separator: {entry!r}")
    type_name, field_name = entry.split("_", 1)
    if not field_name:
        raise MMLogError(f"Invalid header entry with empty field name: {entry!r}")
    if type_name not in TYPE_INFO:
        raise MMLogError(f"Unsupported field type {type_name!r} in header entry {entry!r}")
    struct_code, width = TYPE_INFO[type_name]
    return FieldSpec(type_name=type_name, field_name=field_name, struct_code=struct_code, width=width)


def parse_sidecar(sidecar_path: Path, require_string_hash_metadata: bool) -> SidecarSpec:
    try:
        text = sidecar_path.read_text(encoding="utf-8")
    except FileNotFoundError as exc:
        raise MMLogError(f"Sidecar file not found: {sidecar_path}") from exc

    lines = text.splitlines()

    metadata: Dict[str, str] = {}
    idx = 0
    header_line: str | None = None

    while idx < len(lines):
        line = lines[idx]
        if line == "":
            raise MMLogError(f"Blank line encountered before header in sidecar: {sidecar_path}")
        if "=" in line:
            key, value = line.split("=", 1)
            if key in metadata:
                raise MMLogError(f"Duplicate metadata key {key!r} in sidecar: {sidecar_path}")
            metadata[key] = value
            idx += 1
            continue
        header_line = line
        idx += 1
        break

    if header_line is None:
        raise MMLogError(f"Missing header line in sidecar: {sidecar_path}")

    if "schema_version" not in metadata:
        raise MMLogError(f"Missing required metadata key 'schema_version' in sidecar: {sidecar_path}")
    if "row_bytes" not in metadata:
        raise MMLogError(f"Missing required metadata key 'row_bytes' in sidecar: {sidecar_path}")

    header_entries = [item.strip() for item in header_line.split(",")]
    if not header_entries or any(not item for item in header_entries):
        raise MMLogError(f"Invalid empty header entry in sidecar: {sidecar_path}")

    fields = [parse_header_entry(item) for item in header_entries]
    computed_row_bytes = sum(field.width for field in fields)

    try:
        declared_row_bytes = int(metadata["row_bytes"], 10)
    except ValueError as exc:
        raise MMLogError(f"Invalid integer row_bytes={metadata['row_bytes']!r} in sidecar: {sidecar_path}") from exc

    if computed_row_bytes != declared_row_bytes:
        raise MMLogError(
            f"row_bytes mismatch in {sidecar_path}: header implies {computed_row_bytes}, "
            f"metadata declares {declared_row_bytes}"
        )

    has_string_hash_fields = any(field.is_string_hash for field in fields)
    if has_string_hash_fields:
        if "string_hash" in metadata:
            normalized = normalize_string_hash_metadata(metadata["string_hash"])
            if normalized not in {"fnv1a", "fnv32a"}:
                raise MMLogError(
                    f"Unsupported string_hash={metadata['string_hash']!r} in sidecar {sidecar_path}; "
                    "this decoder only supports FNV-1a"
                )
        elif require_string_hash_metadata:
            raise MMLogError(
                f"Header contains s8/s16/s32 fields but sidecar lacks string_hash metadata: {sidecar_path}"
            )

    labels: List[str] = []
    if idx < len(lines):
        if lines[idx] != "LABELS:":
            raise MMLogError(
                f"Unexpected content after header in sidecar {sidecar_path}: {lines[idx]!r}; "
                "expected 'LABELS:' or end of file"
            )
        idx += 1
        labels = lines[idx:]

    return SidecarSpec(
        metadata=metadata,
        fields=fields,
        labels=labels,
        row_bytes=declared_row_bytes,
    )


def resolve_sidecar_path(mmlog_path: Path, sidecar_ref: str) -> Path:
    ref = Path(sidecar_ref)
    if ref.is_absolute():
        return ref
    return (mmlog_path.parent / ref).resolve()


def read_mmlog_header_and_payload(mmlog_path: Path) -> Tuple[str, bytes]:
    with mmlog_path.open("rb") as f:
        first_line = f.readline()
        if not first_line:
            raise MMLogError(f"Empty .mmlog file: {mmlog_path}")
        try:
            first_line_text = first_line.decode("utf-8").rstrip("\r\n")
        except UnicodeDecodeError as exc:
            raise MMLogError(f"First line is not valid UTF-8 text in {mmlog_path}") from exc

        if not first_line_text.startswith("sidecar_file="):
            raise MMLogError(
                f"First line of {mmlog_path} must begin with 'sidecar_file='; got {first_line_text!r}"
            )

        sidecar_ref = first_line_text.split("=", 1)[1]
        if not sidecar_ref:
            raise MMLogError(f"Empty sidecar_file reference in {mmlog_path}")

        payload = f.read()

    return sidecar_ref, payload


def build_label_maps(labels: Sequence[str], fields: Sequence[FieldSpec]) -> Dict[str, Dict[int, List[str]]]:
    maps: Dict[str, Dict[int, List[str]]] = {}
    for field in fields:
        if not field.is_string_hash:
            continue
        per_hash: Dict[int, List[str]] = {}
        bits = field.hash_bits
        for label in labels:
            h = truncate_hash(fnv1a_32(label.encode("utf-8")), bits)
            per_hash.setdefault(h, []).append(label)
        maps[field.field_name] = per_hash
    return maps


def csv_headers(fields: Sequence[FieldSpec], string_mode: str) -> List[str]:
    headers: List[str] = []
    for field in fields:
        if field.is_string_hash and string_mode == "both":
            headers.append(f"{field.field_name}__raw")
            headers.append(field.field_name)
        elif field.is_string_hash and string_mode == "raw":
            headers.append(field.field_name)
        else:
            headers.append(field.field_name)
    return headers


def decode_hash_value(
    raw_value: int,
    field: FieldSpec,
    label_maps: Dict[str, Dict[int, List[str]]],
    collision_mode: str,
) -> str:
    matches = label_maps.get(field.field_name, {}).get(raw_value, [])
    if not matches:
        return ""

    if len(matches) == 1:
        return matches[0]

    if collision_mode == "join":
        return " | ".join(matches)
    if collision_mode == "first":
        return matches[0]
    raise MMLogError(
        f"Ambiguous label hash for field {field.field_name!r}: value 0x{raw_value:0{field.width * 2}X} "
        f"matches {matches!r}"
    )


def iter_rows(payload: bytes, spec: SidecarSpec) -> Iterable[Tuple[object, ...]]:
    if len(payload) % spec.row_bytes != 0:
        raise MMLogError(
            f"Binary payload size {len(payload)} is not an integer multiple of row_bytes={spec.row_bytes}"
        )
    row_struct = struct.Struct(spec.struct_format)
    for offset in range(0, len(payload), spec.row_bytes):
        yield row_struct.unpack_from(payload, offset)


def decode_to_csv(
    mmlog_path: Path,
    csv_path: Path,
    spec: SidecarSpec,
    payload: bytes,
    string_mode: str,
    collision_mode: str,
) -> int:
    label_maps = build_label_maps(spec.labels, spec.fields)
    headers = csv_headers(spec.fields, string_mode)

    row_count = 0
    with csv_path.open("w", encoding="utf-8", newline="") as f:
        writer = csv.writer(f)
        writer.writerow(headers)

        for row in iter_rows(payload, spec):
            out_row: List[object] = []
            for field, value in zip(spec.fields, row):
                if field.is_string_hash:
                    decoded = decode_hash_value(int(value), field, label_maps, collision_mode)
                    if string_mode == "both":
                        out_row.append(int(value))
                        out_row.append(decoded)
                    elif string_mode == "raw":
                        out_row.append(int(value))
                    elif string_mode == "decoded":
                        out_row.append(decoded)
                    else:
                        raise ValueError(f"Unknown string_mode: {string_mode}")
                else:
                    out_row.append(value)
            writer.writerow(out_row)
            row_count += 1

    return row_count


def find_mmlog_files(root: Path) -> List[Path]:
    if root.is_file():
        return [root] if root.suffix.lower() == ".mmlog" else []

    files: List[Path] = []
    for path in root.rglob("*"):
        if path.is_file() and path.suffix.lower() == ".mmlog":
            files.append(path)
    return sorted(files)


def decode_one_file(
    mmlog_path: Path,
    output_dir: Path | None,
    root: Path,
    string_mode: str,
    collision_mode: str,
    require_string_hash_metadata: bool,
    overwrite: bool,
) -> Tuple[Path, int]:
    sidecar_ref, payload = read_mmlog_header_and_payload(mmlog_path)
    sidecar_path = resolve_sidecar_path(mmlog_path, sidecar_ref)
    spec = parse_sidecar(sidecar_path, require_string_hash_metadata=require_string_hash_metadata)

    if output_dir is None:
        csv_path = mmlog_path.with_suffix(".csv")
    else:
        if root.is_dir():
            try:
                relative_parent = mmlog_path.parent.resolve().relative_to(root.resolve())
            except ValueError:
                relative_parent = Path()
            csv_path = output_dir / relative_parent / f"{mmlog_path.stem}.csv"
        else:
            csv_path = output_dir / f"{mmlog_path.stem}.csv"
        csv_path.parent.mkdir(parents=True, exist_ok=True)

    if csv_path.exists() and not overwrite:
        raise MMLogError(f"Refusing to overwrite existing CSV: {csv_path}")

    row_count = decode_to_csv(
        mmlog_path=mmlog_path,
        csv_path=csv_path,
        spec=spec,
        payload=payload,
        string_mode=string_mode,
        collision_mode=collision_mode,
    )
    return csv_path, row_count


def build_arg_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description="Scan for .mmlog files and decode them to CSV according to the bound .sidecar schema."
    )
    p.add_argument(
        "root",
        nargs="?",
        default=r"D:\\",
        help=r"Root directory or single .mmlog file to process. Default: D:\\",
    )
    p.add_argument(
        "--output-dir",
        type=Path,
        default=None,
        help="Optional directory where CSV files will be written. Default: next to each .mmlog file.",
    )
    p.add_argument(
        "--string-mode",
        choices=("both", "decoded", "raw"),
        default="both",
        help="How to represent s8/s16/s32 fields in the CSV. Default: both",
    )
    p.add_argument(
        "--collision-mode",
        choices=("error", "join", "first"),
        default="join",
        help="How to handle label-hash collisions. Default: join",
    )
    p.add_argument(
        "--require-string-hash-metadata",
        action="store_true",
        help=(
            "Enforce presence of sidecar metadata 'string_hash=...' when s8/s16/s32 fields exist. "
            "Disabled by default because RevG also fixes the hash algorithm to FNV-1a."
        ),
    )
    p.add_argument(
        "--no-overwrite",
        action="store_true",
        help="Do not overwrite existing CSV files.",
    )
    return p


def main(argv: Sequence[str] | None = None) -> int:
    args = build_arg_parser().parse_args(argv)

    root = Path(args.root)
    output_dir = args.output_dir

    if output_dir is not None:
        output_dir.mkdir(parents=True, exist_ok=True)

    mmlog_files = find_mmlog_files(root)
    if not mmlog_files:
        print(f"No .mmlog files found under {root}", file=sys.stderr)
        return 1

    failures = 0
    for mmlog_path in mmlog_files:
        try:
            csv_path, row_count = decode_one_file(
                mmlog_path=mmlog_path,
                output_dir=output_dir,
                root=root,
                string_mode=args.string_mode,
                collision_mode=args.collision_mode,
                require_string_hash_metadata=args.require_string_hash_metadata,
                overwrite=not args.no_overwrite,
            )
            print(f"[OK] {mmlog_path} -> {csv_path} ({row_count} rows)")
        except Exception as exc:
            failures += 1
            print(f"[ERROR] {mmlog_path}: {exc}", file=sys.stderr)

    return 1 if failures else 0


if __name__ == "__main__":
    raise SystemExit(main())
