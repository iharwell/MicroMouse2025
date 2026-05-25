#!/usr/bin/env python3
"""Generate the MazeMap class/struct/free-function dependency graph.

The graph is derived from the newest MSVC `.tlog` directory for
`MazeMap/MazeMap`. `Cl.items.tlog` selects compiled translation units, and
`CL.read.1.tlog` selects the active source/header dependency closure.

The script intentionally uses only the Python standard library.
"""

from __future__ import annotations

import argparse
import re
from collections import Counter
from collections import defaultdict
from dataclasses import dataclass
from dataclasses import field
from pathlib import Path


LOCAL_EXTS = {".h", ".hpp", ".hh", ".hxx", ".cpp", ".cc", ".cxx", ".ino"}
SKIP_DIR_NAMES = {"x64", ".vs"}

CONTROL_WORDS = {
    "if",
    "for",
    "while",
    "switch",
    "catch",
    "return",
    "sizeof",
    "static_cast",
    "reinterpret_cast",
    "const_cast",
    "dynamic_cast",
    "else",
    "delete",
    "new",
    "do",
    "case",
    "static_assert",
    "alignas",
}

CPP_KEYWORDS = {
    "alignas",
    "alignof",
    "and",
    "and_eq",
    "asm",
    "auto",
    "bitand",
    "bitor",
    "bool",
    "break",
    "case",
    "catch",
    "char",
    "char8_t",
    "char16_t",
    "char32_t",
    "class",
    "compl",
    "concept",
    "const",
    "consteval",
    "constexpr",
    "constinit",
    "const_cast",
    "continue",
    "co_await",
    "co_return",
    "co_yield",
    "decltype",
    "default",
    "delete",
    "do",
    "double",
    "dynamic_cast",
    "else",
    "enum",
    "explicit",
    "export",
    "extern",
    "false",
    "float",
    "for",
    "friend",
    "goto",
    "if",
    "inline",
    "int",
    "long",
    "mutable",
    "namespace",
    "new",
    "noexcept",
    "not",
    "not_eq",
    "nullptr",
    "operator",
    "or",
    "or_eq",
    "private",
    "protected",
    "public",
    "register",
    "reinterpret_cast",
    "requires",
    "return",
    "short",
    "signed",
    "sizeof",
    "static",
    "static_assert",
    "static_cast",
    "struct",
    "switch",
    "template",
    "this",
    "thread_local",
    "throw",
    "true",
    "try",
    "typedef",
    "typeid",
    "typename",
    "union",
    "unsigned",
    "using",
    "virtual",
    "void",
    "volatile",
    "wchar_t",
    "while",
    "xor",
    "xor_eq",
}


@dataclass(frozen=True)
class NamespaceRange:
    start: int
    end: int
    names: tuple[str, ...]
    anonymous: bool


@dataclass
class Node:
    kind: str
    qname: str
    simple: str
    file: Path
    relfile: str
    start: int
    end: int
    segment: str = ""
    segments: list[str] = field(default_factory=list)
    declaration_only: bool = False

    def add_segment(self, text: str) -> None:
        if text:
            self.segments.append(text)


@dataclass
class BuildSummary:
    tlog_dir: Path
    output_path: Path
    compiled_source_count: int
    block_count: int
    matched_block_count: int
    active_file_count: int
    class_count: int
    struct_count: int
    function_count: int
    edge_count: int
    weight_counts: Counter[str]
    kind_counts: Counter[tuple[str, str]]
    missing_item_sources: list[str]


def edge_weight(source_kind: str, target_kind: str) -> str:
    if source_kind == "class" and target_kind == "class":
        return "0.2"
    if source_kind == "class" and target_kind in {"function", "struct"}:
        return "1"
    if source_kind in {"function", "struct"} and target_kind == "class":
        return "0.1"
    if source_kind == "struct" and target_kind == "struct":
        return "0.2"
    if source_kind == "function" and target_kind == "function":
        return "0.2"
    if source_kind == "function" and target_kind == "struct":
        return "0.5"
    return "1"


def decode_text(path: Path) -> str:
    data = path.read_bytes()
    if data.startswith(b"\xff\xfe"):
        return data[2:].decode("utf-16-le", errors="ignore")
    if data.startswith(b"\xfe\xff"):
        return data[2:].decode("utf-16-be", errors="ignore")
    if b"\x00" in data[:512]:
        encoding = "utf-16-le" if len(data) > 1 and data[1] == 0 else "utf-16-be"
        return data.decode(encoding, errors="ignore")
    return data.decode("utf-8-sig", errors="ignore")


def norm_path(path: Path) -> str:
    return str(path.resolve()).lower()


def project_relative(path: Path, project_dir: Path) -> str:
    return path.resolve().relative_to(project_dir.resolve()).as_posix()


def latest_tlog_dir(project_dir: Path) -> Path:
    candidates = [path for path in project_dir.rglob("MazeMap.tlog") if path.is_dir()]
    if not candidates:
        raise FileNotFoundError(f"No MazeMap.tlog directory found under {project_dir}")

    def latest_stamp(path: Path) -> float:
        latest = path.stat().st_mtime
        for child in path.rglob("*.tlog"):
            try:
                latest = max(latest, child.stat().st_mtime)
            except FileNotFoundError:
                pass
        return latest

    return max(candidates, key=latest_stamp)


def is_project_file(path: Path, project_dir: Path) -> bool:
    try:
        resolved = path.resolve()
        resolved.relative_to(project_dir.resolve())
    except (FileNotFoundError, ValueError):
        return False
    return (
        resolved.suffix.lower() in LOCAL_EXTS
        and not any(part in SKIP_DIR_NAMES for part in resolved.parts)
    )


def strip_comments_and_literals(text: str) -> str:
    out: list[str] = []
    i = 0
    n = len(text)
    while i < n:
        ch = text[i]
        nxt = text[i + 1] if i + 1 < n else ""
        if ch == "/" and nxt == "/":
            out.extend("  ")
            i += 2
            while i < n and text[i] not in "\r\n":
                out.append(" ")
                i += 1
            continue
        if ch == "/" and nxt == "*":
            out.extend("  ")
            i += 2
            while i < n - 1 and not (text[i] == "*" and text[i + 1] == "/"):
                out.append("\n" if text[i] in "\r\n" else " ")
                i += 1
            if i < n - 1:
                out.extend("  ")
                i += 2
            continue
        if ch == "R" and nxt == '"':
            raw_match = re.match(r'R"([^\s()\\]{0,16})\(', text[i:])
            if raw_match:
                end_token = ")" + raw_match.group(1) + '"'
                end = text.find(end_token, i + len(raw_match.group(0)))
                if end != -1:
                    end += len(end_token)
                    out.extend("\n" if c in "\r\n" else " " for c in text[i:end])
                    i = end
                    continue
        if ch in ('"', "'"):
            quote = ch
            out.append(" ")
            i += 1
            while i < n:
                c = text[i]
                if c == "\\" and i + 1 < n:
                    out.append(" ")
                    out.append("\n" if text[i + 1] in "\r\n" else " ")
                    i += 2
                    continue
                out.append("\n" if c in "\r\n" else " ")
                i += 1
                if c == quote:
                    break
            continue
        out.append(ch)
        i += 1
    return "".join(out)


def blank_preprocessor_definitions(text: str) -> str:
    lines = text.splitlines(keepends=True)
    output: list[str] = []
    in_define = False
    for line in lines:
        stripped = line.lstrip()
        if not in_define and stripped.startswith("#define"):
            in_define = line.rstrip("\r\n").endswith("\\")
            output.append("\n" if line.endswith("\n") else "")
            continue
        if in_define:
            in_define = line.rstrip("\r\n").endswith("\\")
            output.append("\n" if line.endswith("\n") else "")
            continue
        output.append(line)
    return "".join(output)


def find_matching_brace(text: str, open_pos: int) -> int:
    depth = 0
    for i in range(open_pos, len(text)):
        if text[i] == "{":
            depth += 1
        elif text[i] == "}":
            depth -= 1
            if depth == 0:
                return i
    return -1


def find_matching_paren(text: str, open_pos: int) -> int:
    depth = 0
    for i in range(open_pos, len(text)):
        if text[i] == "(":
            depth += 1
        elif text[i] == ")":
            depth -= 1
            if depth == 0:
                return i
    return -1


def build_namespace_ranges(code: str) -> list[NamespaceRange]:
    ranges: list[NamespaceRange] = []
    namespace_re = re.compile(r"\bnamespace\s*(?P<name>[A-Za-z_]\w*(?:::[A-Za-z_]\w*)*)?\s*\{")
    for match in namespace_re.finditer(code):
        open_pos = code.find("{", match.start(), match.end())
        close_pos = find_matching_brace(code, open_pos)
        if close_pos < 0:
            continue
        name = match.group("name")
        ranges.append(
            NamespaceRange(
                match.start(),
                close_pos,
                tuple(part for part in name.split("::") if part) if name else (),
                name is None,
            )
        )
    return ranges


def namespace_scope_at(ranges: list[NamespaceRange], pos: int) -> tuple[tuple[str, ...], bool]:
    names: list[str] = []
    anonymous = False
    containing = sorted((r for r in ranges if r.start <= pos <= r.end), key=lambda r: r.start)
    for item in containing:
        names.extend(item.names)
        anonymous = anonymous or item.anonymous
    return tuple(names), anonymous


def qualified_name(scope: tuple[str, ...], name: str) -> str:
    return "::".join(part for part in (*scope, name) if part)


def sanitized_id(prefix: str, name: str) -> str:
    safe = re.sub(r"[^A-Za-z0-9_]", "_", name)
    if safe and safe[0].isdigit():
        safe = "_" + safe
    return f"{prefix}_{safe}"


def dot_quote(text: str) -> str:
    return '"' + text.replace("\\", "\\\\").replace('"', '\\"') + '"'


def remove_ranges(text: str, ranges: list[tuple[int, int]]) -> str:
    chars = list(text)
    for start, end in ranges:
        for i in range(start, min(end + 1, len(chars))):
            chars[i] = "\n" if chars[i] in "\r\n" else " "
    return "".join(chars)


def source_from_tlog_line(line: str) -> Path | None:
    line = line.strip()
    if not line or line.startswith("#"):
        return None
    source = line.split(";", 1)[0].strip()
    return Path(source) if source else None


def macro_invocation_segment(code: str, start: int) -> tuple[int, int]:
    open_pos = code.find("(", start)
    if open_pos < 0:
        return start, start
    close_pos = find_matching_paren(code, open_pos)
    if close_pos < 0:
        return start, min(len(code) - 1, start + 500)
    semi = code.find(";", close_pos)
    if semi != -1 and semi <= close_pos + 10:
        return start, semi
    return start, close_pos


def suffix_type_lookup(type_qnames: set[str]) -> dict[str, str]:
    lookup: dict[str, str] = {}
    ambiguous: set[str] = set()
    for qname in type_qnames:
        parts = qname.split("::")
        for i in range(len(parts)):
            suffix = "::".join(parts[i:])
            if suffix in lookup and lookup[suffix] != qname:
                ambiguous.add(suffix)
            else:
                lookup[suffix] = qname
    for suffix in ambiguous:
        lookup.pop(suffix, None)
    return lookup


def leading_word(text: str) -> str:
    match = re.search(r"[A-Za-z_]\w*", text)
    return match.group(0) if match else ""


def label_for_node(node: Node) -> str:
    if node.kind == "function" and node.qname.startswith("anonymous::"):
        return node.qname.replace("anonymous::", "")
    return node.qname


def qualified_suffixes(qname: str) -> list[str]:
    parts = qname.split("::")
    suffixes: list[str] = []
    for i in range(max(0, len(parts) - 5), len(parts) - 1):
        suffix = "::".join(parts[i:])
        if "::" in suffix:
            suffixes.append(suffix)
    return sorted(set(suffixes), key=len, reverse=True)


def load_active_files(project_dir: Path, tlog_dir: Path) -> tuple[list[Path], int, int, int, list[str]]:
    items_path = tlog_dir / "Cl.items.tlog"
    read_path = tlog_dir / "CL.read.1.tlog"
    if not items_path.exists() or not read_path.exists():
        raise FileNotFoundError(f"Missing Cl.items.tlog or CL.read.1.tlog under {tlog_dir}")

    compiled_sources: set[str] = set()
    compiled_source_paths: list[Path] = []
    missing_item_sources: list[str] = []
    for line in decode_text(items_path).splitlines():
        source = source_from_tlog_line(line)
        if source and source.exists() and is_project_file(source, project_dir):
            compiled_sources.add(norm_path(source))
            compiled_source_paths.append(source.resolve())
        elif source:
            missing_item_sources.append(str(source))

    blocks: dict[str, list[str]] = defaultdict(list)
    current: str | None = None
    block_count = 0
    matched_block_count = 0
    for raw_line in decode_text(read_path).splitlines():
        line = raw_line.strip()
        if not line:
            continue
        if line.startswith("^"):
            source_text = line[1:].strip()
            source_path = Path(source_text)
            current = norm_path(source_path) if source_path.exists() else source_text.lower()
            block_count += 1
            if current in compiled_sources:
                matched_block_count += 1
            continue
        if current is None:
            continue
        dep = Path(line)
        if dep.exists() and is_project_file(dep, project_dir):
            blocks[current].append(norm_path(dep))

    active_file_keys = set(compiled_sources)
    for source_key, deps in blocks.items():
        if source_key in compiled_sources:
            active_file_keys.update(deps)

    all_project_files: dict[str, Path] = {}
    for path in project_dir.rglob("*"):
        if path.is_file() and is_project_file(path, project_dir):
            all_project_files[norm_path(path)] = path.resolve()

    active_files = sorted(
        (all_project_files[key] for key in active_file_keys if key in all_project_files),
        key=lambda path: project_relative(path, project_dir).lower(),
    )
    return active_files, len(compiled_source_paths), block_count, matched_block_count, missing_item_sources


def collect_file_data(project_dir: Path, active_files: list[Path]) -> dict[Path, dict[str, object]]:
    file_data: dict[Path, dict[str, object]] = {}
    for path in active_files:
        raw = decode_text(path)
        code = strip_comments_and_literals(blank_preprocessor_definitions(raw))
        dep_code = strip_comments_and_literals(raw)
        file_data[path] = {
            "raw": raw,
            "code": code,
            "dep_code": dep_code,
            "namespaces": build_namespace_ranges(code),
            "relfile": project_relative(path, project_dir),
        }
    return file_data


def collect_types(
    file_data: dict[Path, dict[str, object]],
) -> tuple[dict[tuple[str, str], Node], dict[Path, list[tuple[int, int, str]]]]:
    nodes: dict[tuple[str, str], Node] = {}
    type_ranges_by_file: dict[Path, list[tuple[int, int, str]]] = defaultdict(list)
    all_type_defs_by_file: dict[Path, list[Node]] = defaultdict(list)
    type_re = re.compile(
        r"\b(?P<kind>class|struct)\s+"
        r"(?:(?:EXPORT|MAZEMAP_API|MMLOG_PACKED|__declspec\s*\([^)]*\)|alignas\s*\([^)]*\))\s+)*"
        r"(?P<name>[A-Za-z_]\w*)\b"
        r"(?P<trailer>\s*(?:final\s*)?(?::[^;{}]*)?)\{",
        re.MULTILINE | re.DOTALL,
    )

    pending_type_defs: list[Node] = []
    for path, data in file_data.items():
        code = data["code"]
        assert isinstance(code, str)
        for match in type_re.finditer(code):
            name = match.group("name")
            if name in {"EntryName", "RowName"} or name in CPP_KEYWORDS:
                continue
            open_pos = code.find("{", match.start(), match.end())
            close_pos = find_matching_brace(code, open_pos)
            if close_pos < 0:
                continue
            relfile = data["relfile"]
            assert isinstance(relfile, str)
            node = Node(
                match.group("kind"),
                name,
                name,
                path,
                relfile,
                match.start(),
                close_pos,
                code[match.start() : close_pos + 1],
            )
            pending_type_defs.append(node)
            all_type_defs_by_file[path].append(node)
            type_ranges_by_file[path].append((match.start(), close_pos, name))

    for node in pending_type_defs:
        namespaces = file_data[node.file]["namespaces"]
        assert isinstance(namespaces, list)
        ns_names, _ = namespace_scope_at(namespaces, node.start)
        parents = [
            other
            for other in all_type_defs_by_file[node.file]
            if other.start < node.start and node.end < other.end
        ]
        parents.sort(key=lambda item: item.start)
        qname = qualified_name((*ns_names, *(parent.simple for parent in parents)), node.simple)
        key = (node.kind, qname)
        if key in nodes:
            nodes[key].add_segment(node.segment)
        else:
            node.qname = qname
            node.add_segment(node.segment)
            nodes[key] = node

    mmlog_re = re.compile(
        r"\bMMLOG_DEFINE(?:_PRIVATE)?_(?:ENTRY|ROW)(?:_WITH_BODY)?\s*\(\s*(?P<name>[A-Za-z_]\w*)",
        re.MULTILINE,
    )
    for path, data in file_data.items():
        dep_code = data["dep_code"]
        namespaces = data["namespaces"]
        relfile = data["relfile"]
        assert isinstance(dep_code, str)
        assert isinstance(namespaces, list)
        assert isinstance(relfile, str)
        for match in mmlog_re.finditer(dep_code):
            name = match.group("name")
            ns_names, _ = namespace_scope_at(namespaces, match.start())
            qname = qualified_name(ns_names, name)
            seg_start, seg_end = macro_invocation_segment(dep_code, match.start())
            segment = dep_code[seg_start : seg_end + 1]
            key = ("struct", qname)
            if key not in nodes:
                node = Node("struct", qname, name, path, relfile, match.start(), seg_end, segment)
                node.add_segment(segment)
                nodes[key] = node
            else:
                nodes[key].add_segment(segment)

    return nodes, type_ranges_by_file


def collect_functions(
    file_data: dict[Path, dict[str, object]],
    nodes: dict[tuple[str, str], Node],
    type_ranges_by_file: dict[Path, list[tuple[int, int, str]]],
) -> None:
    type_qnames = {qname for (kind, qname) in nodes if kind in {"class", "struct"}}
    type_lookup = suffix_type_lookup(type_qnames)
    simple_type_names = {node.simple for node in nodes.values() if node.kind in {"class", "struct"}}
    function_nodes: dict[str, Node] = {}
    member_segments: dict[str, list[str]] = defaultdict(list)
    function_start_re = re.compile(
        r"(?m)^[ \t]*(?:template\s*<[^;{}]*>\s*)?"
        r"(?:(?:EXPORT|static|inline|constexpr|consteval|extern)\s+)*"
        r"(?:[A-Za-z_~][\w:<>,\s*&\[\]]+?\s+)"
        r"(?P<name>(?:[A-Za-z_]\w*::)*[A-Za-z_]\w*|operator\s*[^\s(]+)\s*\("
    )

    for path, data in file_data.items():
        code = data["code"]
        namespaces = data["namespaces"]
        relfile = data["relfile"]
        assert isinstance(code, str)
        assert isinstance(namespaces, list)
        assert isinstance(relfile, str)
        no_types = remove_ranges(
            code,
            [(start, end) for start, end, _ in type_ranges_by_file[path]],
        )
        for match in function_start_re.finditer(no_types):
            sig_start = match.start()
            line_start = no_types.rfind("\n", 0, sig_start) + 1
            signature_prefix = no_types[line_start : match.start("name")]
            lead = leading_word(signature_prefix)
            if lead in CONTROL_WORDS or not lead:
                continue
            if "=" in signature_prefix and "operator" not in match.group("name"):
                continue

            open_pos = no_types.find("(", match.start("name"), match.end())
            close_paren = find_matching_paren(no_types, open_pos)
            if close_paren < 0:
                continue

            cursor = close_paren + 1
            while cursor < len(no_types):
                c = no_types[cursor]
                if c in "{;":
                    break
                if c == "=" and no_types[cursor : cursor + 12].strip().startswith(
                    ("= delete", "= default")
                ):
                    semi = no_types.find(";", cursor)
                    if semi != -1:
                        cursor = semi
                        break
                cursor += 1
            if cursor >= len(no_types) or no_types[cursor] not in "{;":
                continue

            declaration_only = no_types[cursor] == ";"
            sig_text = no_types[line_start : cursor + 1]
            if declaration_only and not re.search(r"\b(EXPORT|inline|constexpr|consteval)\b", sig_text):
                continue
            end_pos = cursor if declaration_only else find_matching_brace(no_types, cursor)
            if end_pos < 0:
                continue

            segment = no_types[line_start : end_pos + 1]
            raw_name = match.group("name").replace(" ", "")
            simple_candidate = raw_name.split("::")[-1]
            if (
                not raw_name
                or simple_candidate in CONTROL_WORDS
                or simple_candidate in CPP_KEYWORDS
            ):
                continue
            if simple_candidate in simple_type_names and "::" not in raw_name:
                continue

            ns_names, anonymous = namespace_scope_at(namespaces, sig_start)
            is_static = bool(re.search(r"\bstatic\b", signature_prefix))
            owner_qname = None
            if "::" in raw_name:
                owner_suffix = raw_name.rsplit("::", 1)[0]
                candidates = [owner_suffix]
                if ns_names:
                    candidates.append("::".join((*ns_names, owner_suffix)))
                for candidate in candidates:
                    if candidate in type_qnames:
                        owner_qname = candidate
                        break
                    if candidate in type_lookup:
                        owner_qname = type_lookup[candidate]
                        break
            if owner_qname:
                member_segments[owner_qname].append(segment)
                continue

            simple = raw_name if raw_name.startswith("operator") else simple_candidate
            qname = raw_name if "::" in raw_name else qualified_name(ns_names, raw_name)
            if anonymous or is_static:
                file_scope = relfile.replace("/", "::").replace(".", "_")
                qname = f"anonymous::{file_scope}::{simple}"
            if qname not in function_nodes:
                function_nodes[qname] = Node(
                    "function",
                    qname,
                    simple,
                    path,
                    relfile,
                    sig_start,
                    end_pos,
                    segment,
                    declaration_only=declaration_only,
                )
            function_nodes[qname].add_segment(segment)

    for qname, segments in member_segments.items():
        for kind in ("class", "struct"):
            key = (kind, qname)
            if key in nodes:
                for segment in segments:
                    nodes[key].add_segment(segment)
                break

    for qname, node in function_nodes.items():
        nodes[("function", qname)] = node


def build_edges(nodes: dict[tuple[str, str], Node]) -> set[tuple[tuple[str, str], tuple[str, str]]]:
    simple_to_nodes: dict[str, list[tuple[str, str]]] = defaultdict(list)
    for key, node in nodes.items():
        if node.simple not in CPP_KEYWORDS:
            simple_to_nodes[node.simple].append(key)

    unique_simple_targets = {
        simple: keys[0] for simple, keys in simple_to_nodes.items() if len(keys) == 1
    }
    duplicate_targets = {
        simple: keys for simple, keys in simple_to_nodes.items() if len(keys) > 1
    }
    target_suffixes = {
        key: qualified_suffixes(nodes[key].qname)
        for keys in duplicate_targets.values()
        for key in keys
    }

    edges: set[tuple[tuple[str, str], tuple[str, str]]] = set()
    identifier_re = re.compile(r"\b[A-Za-z_]\w*\b")
    qualified_re = re.compile(r"\b[A-Za-z_]\w*(?:::[A-Za-z_]\w*)+\b")
    for source_key, source_node in nodes.items():
        source_text = "\n".join(source_node.segments)
        if not source_text:
            continue
        tokens = set(identifier_re.findall(source_text))
        qualified_mentions = set(qualified_re.findall(source_text))
        for simple in tokens:
            target_key = unique_simple_targets.get(simple)
            if target_key and target_key != source_key:
                edges.add((source_key, target_key))
            for duplicate_target in duplicate_targets.get(simple, []):
                if duplicate_target == source_key:
                    continue
                if any(suffix in qualified_mentions for suffix in target_suffixes[duplicate_target]):
                    edges.add((source_key, duplicate_target))
    return edges


def assign_node_ids(nodes: dict[tuple[str, str], Node]) -> dict[tuple[str, str], str]:
    node_ids: dict[tuple[str, str], str] = {}
    for key, node in sorted(nodes.items(), key=lambda item: (item[0][0], item[1].qname.lower())):
        prefix = {"class": "class", "struct": "struct", "function": "fn"}[node.kind]
        candidate = sanitized_id(prefix, node.qname)
        base = candidate
        i = 2
        while candidate in node_ids.values():
            candidate = f"{base}_{i}"
            i += 1
        node_ids[key] = candidate
    return node_ids


def write_dot(
    root: Path,
    project_dir: Path,
    output_path: Path,
    tlog_dir: Path,
    active_file_count: int,
    compiled_source_count: int,
    block_count: int,
    matched_block_count: int,
    missing_item_sources: list[str],
    nodes: dict[tuple[str, str], Node],
    edges: set[tuple[tuple[str, str], tuple[str, str]]],
) -> None:
    node_ids = assign_node_ids(nodes)
    class_count = sum(1 for node in nodes.values() if node.kind == "class")
    struct_count = sum(1 for node in nodes.values() if node.kind == "struct")
    function_count = sum(1 for node in nodes.values() if node.kind == "function")

    lines: list[str] = []
    lines.append("digraph MazeMapClassStructDependencies {")
    lines.append(f"  // Generated from {tlog_dir.relative_to(root).as_posix()}")
    lines.append(
        f"  // Compiler items: {compiled_source_count} existing sources; "
        f"CL.read blocks: {block_count}; matched blocks: {matched_block_count}"
    )
    lines.append(f"  // Active local dependency files: {active_file_count}")
    lines.append(
        f"  // Nodes: {class_count} classes, {struct_count} structs, "
        f"{function_count} unowned functions; edges: {len(edges)}"
    )
    lines.append(
        "  // Edge weights: class->class 0.2; class->function/struct 1; "
        "function/struct->class 0.1; struct->struct and function->function 0.2; "
        "function->struct 0.5; unspecified struct->function 1."
    )
    lines.append(
        "  // Edges mean the source node's declaration or implementation references the target node name."
    )
    if missing_item_sources:
        lines.append(f"  // Cl.items entries missing from disk and skipped: {len(missing_item_sources)}")
    lines.append('  graph [rankdir=LR, overlap=false, splines=true, fontname="Consolas"];')
    lines.append('  node [fontname="Consolas", fontsize=10, style=filled];')
    lines.append('  edge [color=gray45, arrowsize=0.7];')
    lines.append("")
    lines.append("  subgraph cluster_legend {")
    lines.append('    label="Legend";')
    lines.append("    color=gray70;")
    lines.append(
        '    legend_class [label="class", shape=box, fillcolor=black, color=black, fontcolor=white];'
    )
    lines.append(
        '    legend_struct [label="struct", shape=oval, fillcolor=red, color=red, fontcolor=white];'
    )
    lines.append(
        '    legend_function [label="unowned function", shape=diamond, fillcolor=blue, color=blue, fontcolor=white];'
    )
    lines.append("  }")
    lines.append("")

    for key, node in sorted(nodes.items(), key=lambda item: (item[1].kind, item[1].qname.lower())):
        attrs = {"label": label_for_node(node), "tooltip": node.relfile}
        if node.kind == "class":
            attrs.update({"shape": "box", "fillcolor": "black", "color": "black", "fontcolor": "white"})
        elif node.kind == "struct":
            attrs.update({"shape": "oval", "fillcolor": "red", "color": "red", "fontcolor": "white"})
        else:
            attrs.update(
                {"shape": "diamond", "fillcolor": "blue", "color": "blue", "fontcolor": "white"}
            )
        attr_text = ", ".join(f"{name}={dot_quote(value)}" for name, value in attrs.items())
        lines.append(f"  {node_ids[key]} [{attr_text}];")

    lines.append("")
    for source_key, target_key in sorted(
        edges,
        key=lambda edge: (nodes[edge[0]].qname.lower(), nodes[edge[1]].qname.lower()),
    ):
        weight = edge_weight(nodes[source_key].kind, nodes[target_key].kind)
        lines.append(f"  {node_ids[source_key]} -> {node_ids[target_key]} [weight={weight}];")
    lines.append("}")

    output_path.parent.mkdir(parents=True, exist_ok=True)
    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")


def build_graph(project_dir: Path, output_path: Path, tlog_dir: Path | None) -> BuildSummary:
    root = Path(__file__).resolve().parents[1]
    resolved_tlog_dir = tlog_dir.resolve() if tlog_dir else latest_tlog_dir(project_dir)
    (
        active_files,
        compiled_source_count,
        block_count,
        matched_block_count,
        missing_item_sources,
    ) = load_active_files(project_dir, resolved_tlog_dir)
    file_data = collect_file_data(project_dir, active_files)
    nodes, type_ranges_by_file = collect_types(file_data)
    collect_functions(file_data, nodes, type_ranges_by_file)
    edges = build_edges(nodes)
    write_dot(
        root,
        project_dir,
        output_path,
        resolved_tlog_dir,
        len(active_files),
        compiled_source_count,
        block_count,
        matched_block_count,
        missing_item_sources,
        nodes,
        edges,
    )

    weight_counts = Counter(edge_weight(nodes[source].kind, nodes[target].kind) for source, target in edges)
    kind_counts = Counter((nodes[source].kind, nodes[target].kind) for source, target in edges)
    return BuildSummary(
        resolved_tlog_dir,
        output_path,
        compiled_source_count,
        block_count,
        matched_block_count,
        len(active_files),
        sum(1 for node in nodes.values() if node.kind == "class"),
        sum(1 for node in nodes.values() if node.kind == "struct"),
        sum(1 for node in nodes.values() if node.kind == "function"),
        len(edges),
        weight_counts,
        kind_counts,
        missing_item_sources,
    )


def default_project_dir() -> Path:
    return Path(__file__).resolve().parents[1] / "MazeMap" / "MazeMap"


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser(
        description="Generate the MazeMap class/struct/free-function dependency DOT graph from MSVC tlogs."
    )
    parser.add_argument(
        "--project-dir",
        type=Path,
        default=default_project_dir(),
        help="Path to MazeMap/MazeMap. Defaults to this repository's MazeMap/MazeMap.",
    )
    parser.add_argument(
        "--tlog-dir",
        type=Path,
        default=None,
        help="Specific MazeMap.tlog directory to read. Defaults to the newest one under project-dir.",
    )
    parser.add_argument(
        "--output",
        type=Path,
        default=None,
        help="DOT output path. Defaults to MazeMapClassStructDependencies.dot in project-dir.",
    )
    return parser.parse_args()


def main() -> int:
    args = parse_args()
    project_dir = args.project_dir.resolve()
    output_path = args.output.resolve() if args.output else project_dir / "MazeMapClassStructDependencies.dot"
    summary = build_graph(project_dir, output_path, args.tlog_dir)

    print(f"tlog_dir={summary.tlog_dir}")
    print(f"output={summary.output_path}")
    print(f"compiled_existing_sources={summary.compiled_source_count}")
    print(f"cl_read_blocks={summary.block_count}")
    print(f"matched_blocks={summary.matched_block_count}")
    print(f"active_local_files={summary.active_file_count}")
    print(f"classes={summary.class_count}")
    print(f"structs={summary.struct_count}")
    print(f"functions={summary.function_count}")
    print(f"edges={summary.edge_count}")
    print(
        "weight_counts="
        + ", ".join(
            f"{weight}:{summary.weight_counts[weight]}"
            for weight in sorted(summary.weight_counts, key=float)
        )
    )
    print(f"struct_function_edges={summary.kind_counts[('struct', 'function')]}")
    if summary.missing_item_sources:
        print("missing_item_sources=" + ";".join(summary.missing_item_sources))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
