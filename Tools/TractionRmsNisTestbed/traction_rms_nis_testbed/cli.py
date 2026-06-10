"""CLI for exporting traction RMS/NIS observable streams."""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path
from typing import Iterable

from .data_layer import (
    DEFAULT_MANIFEST_PATH,
    DEFAULT_OUTPUT_DIR,
    TractionObservableDataLayer,
)


def parse_args(argv: Iterable[str]) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--manifest",
        default=str(DEFAULT_MANIFEST_PATH),
        help="Path to segment_manifest.json.",
    )
    parser.add_argument(
        "--output-dir",
        default=str(DEFAULT_OUTPUT_DIR),
        help="Directory for segment_streams.jsonl and summary.json.",
    )
    parser.add_argument(
        "--repo-root",
        default="",
        help="Optional repository root override for relative log paths.",
    )
    parser.add_argument(
        "--segment-id",
        action="append",
        default=[],
        help="Segment id to export. May be supplied more than once.",
    )
    parser.add_argument(
        "--limit",
        type=int,
        default=0,
        help="Maximum number of manifest segments to export after filtering.",
    )
    return parser.parse_args(list(argv))


def main(argv: Iterable[str] | None = None) -> int:
    args = parse_args(sys.argv[1:] if argv is None else argv)
    manifest_path = Path(args.manifest).resolve()
    output_dir = Path(args.output_dir).resolve()
    repo_root = Path(args.repo_root).resolve() if args.repo_root else None
    segment_ids = set(args.segment_id) if args.segment_id else None

    layer = TractionObservableDataLayer(manifest_path=manifest_path, repo_root=repo_root)
    output_dir.mkdir(parents=True, exist_ok=True)

    exported = []
    jsonl_path = output_dir / "segment_streams.jsonl"
    with jsonl_path.open("w", encoding="utf-8", newline="\n") as handle:
        for stream in layer.iter_segment_streams(
            segment_ids=segment_ids,
            limit=args.limit or None,
        ):
            payload = stream.to_json_dict()
            handle.write(json.dumps(payload, separators=(",", ":"), sort_keys=True))
            handle.write("\n")
            exported.append(
                {
                    "segment_id": stream.definition.segment_id,
                    "log_path": stream.definition.log_path,
                    "row_count": stream.row_count,
                    "stage": stream.definition.stage,
                    "family": stream.definition.family,
                    "corrupted": stream.boundaries["corruption"]["is_corrupted"],
                    "stream_groups": sorted(stream.streams.keys()),
                }
            )

    summary = {
        "manifest_path": str(manifest_path),
        "output_jsonl": str(jsonl_path),
        "segments_exported": len(exported),
        "source_logs": sorted({item["log_path"] for item in exported}),
        "ignored_state_column_prefixes": layer.ignored_state_column_prefixes,
        "segments": exported,
    }
    (output_dir / "summary.json").write_text(
        json.dumps(summary, indent=2, sort_keys=True) + "\n",
        encoding="utf-8",
    )
    print(f"Wrote {len(exported)} segment observable stream(s) to {jsonl_path}")
    return 0
