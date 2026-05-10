from __future__ import annotations

import argparse
import json
import re
import subprocess
import sys
from dataclasses import dataclass
from pathlib import Path
from typing import Iterable


CPP_EXTENSIONS = {
    ".c",
    ".cc",
    ".cpp",
    ".h",
    ".hh",
    ".hpp",
    ".ino",
    ".inl",
    ".ipp",
}

EXCLUDED_PATH_FRAGMENTS = (
    "/.git/",
    "/.tmp/",
    "/.vs/",
    "/DriveProgramFiles/",
    "/Maze Files/",
    "/TestResults/",
    "/codex_temp/",
    "/codex_verify/arduino_build/",
    "/codex_verify/arduino_build_work_",
    "/codex_verify/arduino_libraries/Eigen/",
    "/codex_verify/isolated_release",
    "/MazeMap/eigen-",
    "/MazeMap/MazeSimulation/x64/",
    "/scripts/__pycache__/",
)

BOUNDARY_FILES = {
    "MazeMap/MazeMap/Defines.h",
    "MazeMap/MazeMap/Defines.cpp",
}

RAW_LOGGING_OWNER_FILES = {
    "MazeMap/MazeMap/SharedRobotRuntime.h",
    "MazeMap/MazeMap/SharedRobotRuntime.cpp",
    "MazeMap/MazeMap/MmLog.h",
    "MazeMap/MazeMap/MmLog.cpp",
    "MazeMap/MazeMap/MazeMapRuntimeMmLog.h",
    "MazeMap/MazeMap/MazeMapRuntimeInfrastructure.h",
    "MazeMap/MazeMap/MazeMapRuntimeInfrastructure.cpp",
}

SUSPICIOUS_TYPE_SUFFIXES = (
    "Context",
    "Params",
    "Parameters",
    "State",
    "Data",
    "Info",
    "Snapshot",
    "Bundle",
    "Record",
    "Helper",
    "Manager",
    "Facade",
    "Utils",
    "Support",
)

CONTROL_KEYWORDS = {
    "catch",
    "class",
    "do",
    "else",
    "enum",
    "for",
    "if",
    "namespace",
    "return",
    "sizeof",
    "static_assert",
    "struct",
    "switch",
    "try",
    "while",
}

SEVERITY_ORDER = {
    "P0": 0,
    "P1": 1,
    "P2": 2,
}


@dataclass(frozen=True)
class ScopeView:
    kind: str
    name: str
    access: str


@dataclass
class Scope:
    kind: str
    name: str
    access: str


@dataclass(frozen=True)
class Finding:
    severity: str
    rule: str
    path: Path
    line: int
    message: str
    remediation: str


@dataclass(frozen=True)
class FunctionDefinition:
    name: str
    start_line: int
    end_line: int
    body: str


def run_git(root: Path, args: list[str]) -> str:
    return subprocess.check_output(
        ["git", *args],
        cwd=root,
        text=True,
        encoding="utf-8",
        errors="replace",
    )


def normalize_relative_path(path: str | Path) -> Path:
    return Path(str(path).replace("\\", "/"))


def path_key(path: Path) -> str:
    return path.as_posix()


def is_cpp_source(path: Path) -> bool:
    return path.suffix.lower() in CPP_EXTENSIONS


def is_excluded(path: Path) -> bool:
    normalized = "/" + path.as_posix()
    return any(fragment in normalized for fragment in EXCLUDED_PATH_FRAGMENTS)


def should_scan_path(path: Path, include_tests: bool) -> bool:
    if not is_cpp_source(path):
        return False
    if is_excluded(path):
        return False
    if not include_tests and is_test_path(path):
        return False
    return True


def is_production_path(path: Path) -> bool:
    parts = path.parts
    return len(parts) >= 2 and parts[0] == "MazeMap" and parts[1] == "MazeMap"


def is_test_path(path: Path) -> bool:
    parts = path.parts
    return len(parts) >= 2 and parts[0] == "MazeMap" and parts[1] == "MazeMapTest"


def load_tracked_source_files(root: Path, include_tests: bool) -> list[Path]:
    output = run_git(root, ["ls-files", "-z"])
    files = [normalize_relative_path(item) for item in output.split("\0") if item]
    return sorted(path for path in files if should_scan_path(path, include_tests))


def load_untracked_source_files(root: Path, include_tests: bool) -> list[Path]:
    output = run_git(root, ["ls-files", "--others", "--exclude-standard", "-z"])
    files = [normalize_relative_path(item) for item in output.split("\0") if item]
    return sorted(path for path in files if should_scan_path(path, include_tests))


def parse_unified_zero_diff(diff_text: str, include_tests: bool) -> dict[Path, set[int]]:
    changed: dict[Path, set[int]] = {}
    current_path: Path | None = None
    new_line_number: int | None = None

    for line in diff_text.splitlines():
        if line.startswith("diff --git "):
            current_path = None
            new_line_number = None
            continue

        if line.startswith("+++ "):
            if line == "+++ /dev/null":
                current_path = None
                continue
            path_text = line[6:] if line.startswith("+++ b/") else line[4:]
            candidate = normalize_relative_path(path_text)
            current_path = candidate if should_scan_path(candidate, include_tests) else None
            continue

        if line.startswith("@@ "):
            match = re.search(r"\+(\d+)(?:,(\d+))?", line)
            new_line_number = int(match.group(1)) if match else None
            continue

        if current_path is None or new_line_number is None:
            continue

        if line.startswith("+") and not line.startswith("+++"):
            changed.setdefault(current_path, set()).add(new_line_number)
            new_line_number += 1
            continue

        if line.startswith("-") and not line.startswith("---"):
            continue

        if not line.startswith("\\"):
            new_line_number += 1

    return changed


def merge_changed_lines(target: dict[Path, set[int]], source: dict[Path, set[int]]) -> None:
    for path, lines in source.items():
        target.setdefault(path, set()).update(lines)


def load_changed_lines(root: Path, mode: str, base: str, include_tests: bool) -> dict[Path, set[int]]:
    if mode == "staged":
        diff = run_git(root, ["diff", "--cached", "--unified=0", "--no-ext-diff", "--"])
        changed = parse_unified_zero_diff(diff, include_tests)
    else:
        diff = run_git(root, ["diff", "--unified=0", "--no-ext-diff", base, "--"])
        changed = parse_unified_zero_diff(diff, include_tests)

    if mode == "changed":
        for path in load_untracked_source_files(root, include_tests):
            try:
                line_count = len((root / path).read_text(encoding="utf-8", errors="replace").splitlines())
            except OSError:
                continue
            changed[path] = set(range(1, line_count + 1))

    return changed


def strip_cpp_comments_and_strings(source: str) -> str:
    result: list[str] = []
    i = 0
    length = len(source)
    in_line_comment = False
    in_block_comment = False
    in_string = False
    delimiter = ""

    while i < length:
        char = source[i]
        nxt = source[i + 1] if i + 1 < length else ""

        if in_line_comment:
            if char == "\n":
                in_line_comment = False
                result.append("\n")
            else:
                result.append(" ")
            i += 1
            continue

        if in_block_comment:
            if char == "*" and nxt == "/":
                in_block_comment = False
                result.append("  ")
                i += 2
            else:
                result.append("\n" if char == "\n" else " ")
                i += 1
            continue

        if in_string:
            if char == "\\" and i + 1 < length:
                result.append("  ")
                i += 2
                continue
            if char == delimiter:
                in_string = False
            result.append("\n" if char == "\n" else " ")
            i += 1
            continue

        if char == "R" and nxt == '"':
            delimiter_end = source.find("(", i + 2, min(i + 32, length))
            if delimiter_end != -1:
                raw_delimiter = source[i + 2 : delimiter_end]
                closing = ")" + raw_delimiter + '"'
                closing_index = source.find(closing, delimiter_end + 1)
                if closing_index != -1:
                    raw_text = source[i : closing_index + len(closing)]
                    result.extend("\n" if raw_char == "\n" else " " for raw_char in raw_text)
                    i = closing_index + len(closing)
                    continue

        if char == "/" and nxt == "/":
            in_line_comment = True
            result.append("  ")
            i += 2
            continue

        if char == "/" and nxt == "*":
            in_block_comment = True
            result.append("  ")
            i += 2
            continue

        if char in {'"', "'"}:
            in_string = True
            delimiter = char
            result.append(" ")
            i += 1
            continue

        result.append(char)
        i += 1

    return "".join(result)


def line_offsets(source: str) -> list[int]:
    offsets = [0]
    for match in re.finditer("\n", source):
        offsets.append(match.end())
    return offsets


def line_number_for_offset(offsets: list[int], index: int) -> int:
    low = 0
    high = len(offsets)
    while low + 1 < high:
        mid = (low + high) // 2
        if offsets[mid] <= index:
            low = mid
        else:
            high = mid
    return low + 1


def previous_construct_text(clean_source: str, brace_index: int) -> str:
    i = brace_index - 1
    paren_depth = 0
    bracket_depth = 0
    angle_depth = 0

    while i >= 0:
        char = clean_source[i]
        if char == ")":
            paren_depth += 1
        elif char == "(" and paren_depth > 0:
            paren_depth -= 1
        elif char == "]":
            bracket_depth += 1
        elif char == "[" and bracket_depth > 0:
            bracket_depth -= 1
        elif char == ">":
            angle_depth += 1
        elif char == "<" and angle_depth > 0:
            angle_depth -= 1
        elif paren_depth == 0 and bracket_depth == 0 and angle_depth == 0 and char in ";{}":
            return clean_source[i + 1 : brace_index].strip()
        i -= 1

    return clean_source[:brace_index].strip()


def classify_brace_scope(preface: str) -> Scope:
    compact = re.sub(r"\s+", " ", preface).strip()

    type_match = re.search(r"\b(class|struct)\s+([A-Za-z_]\w*)\b", compact)
    if type_match:
        kind = type_match.group(1)
        default_access = "public" if kind == "struct" else "private"
        return Scope(kind=kind, name=type_match.group(2), access=default_access)

    namespace_match = re.fullmatch(r"(?:inline\s+)?namespace(?:\s+([A-Za-z_][\w:]*))?", compact)
    if namespace_match:
        return Scope(kind="namespace", name=namespace_match.group(1) or "<anonymous>", access="public")

    function_name = extract_function_name(compact)
    if function_name:
        return Scope(kind="function", name=function_name, access="private")

    return Scope(kind="block", name="", access="private")


def build_scope_contexts(clean_source: str) -> list[tuple[ScopeView, ...]]:
    contexts: list[tuple[ScopeView, ...]] = [tuple() for _ in range(clean_source.count("\n") + 3)]
    stack: list[Scope] = []

    index = 0
    line_number = 1
    line_start = 0
    length = len(clean_source)

    while index <= length:
        if index == length or clean_source[index] == "\n":
            line_text = clean_source[line_start:index].strip()
            access_match = re.match(r"^(public|private|protected)\s*:", line_text)
            if access_match:
                for scope in reversed(stack):
                    if scope.kind in {"class", "struct"}:
                        scope.access = access_match.group(1)
                        break

            if line_number >= len(contexts):
                contexts.extend(tuple() for _ in range(line_number - len(contexts) + 2))
            contexts[line_number] = tuple(ScopeView(scope.kind, scope.name, scope.access) for scope in stack)
            line_number += 1
            line_start = index + 1
            index += 1
            continue

        char = clean_source[index]
        if char == "{":
            stack.append(classify_brace_scope(previous_construct_text(clean_source, index)))
        elif char == "}":
            if stack:
                stack.pop()
        index += 1

    for empty_line in range(line_number, len(contexts)):
        contexts[empty_line] = tuple(ScopeView(scope.kind, scope.name, scope.access) for scope in stack)

    return contexts


def extract_function_name(signature: str) -> str | None:
    compact = re.sub(r"\s+", " ", signature).strip()
    if not compact or compact.startswith("#"):
        return None

    lowered = compact.lower()
    for keyword in CONTROL_KEYWORDS:
        if lowered.startswith(keyword + " ") or lowered.startswith(keyword + "("):
            return None

    if "(" not in compact or ")" not in compact:
        return None

    open_paren = compact.rfind("(", 0, compact.rfind(")") + 1)
    if open_paren == -1:
        return None

    prefix = compact[:open_paren].rstrip()
    if not prefix:
        return None

    name_match = re.search(
        r"((?:[A-Za-z_~]\w*|operator\s*(?:\[\]|\(\)|[^\s(]+))(?:\s*::\s*(?:[A-Za-z_~]\w*|operator\s*(?:\[\]|\(\)|[^\s(]+)))*)$",
        prefix,
    )
    if not name_match:
        return None

    name = re.sub(r"\s+", "", name_match.group(1).replace("operator", "operator "))
    if name in CONTROL_KEYWORDS:
        return None
    return name


def find_matching_brace(clean_source: str, open_index: int) -> int | None:
    depth = 1
    index = open_index + 1
    while index < len(clean_source):
        char = clean_source[index]
        if char == "{":
            depth += 1
        elif char == "}":
            depth -= 1
            if depth == 0:
                return index
        index += 1
    return None


def find_function_definitions(clean_source: str) -> list[FunctionDefinition]:
    offsets = line_offsets(clean_source)
    definitions: list[FunctionDefinition] = []
    index = 0

    while index < len(clean_source):
        if clean_source[index] != "{":
            index += 1
            continue

        preface = previous_construct_text(clean_source, index)
        name = extract_function_name(preface)
        if not name:
            index += 1
            continue

        end = find_matching_brace(clean_source, index)
        if end is None:
            index += 1
            continue

        definitions.append(
            FunctionDefinition(
                name=name,
                start_line=line_number_for_offset(offsets, index),
                end_line=line_number_for_offset(offsets, end),
                body=clean_source[index + 1 : end],
            )
        )
        index = end + 1

    return definitions


def statement_span_for_line(lines: list[str], line_index: int, max_radius: int = 8) -> tuple[int, int, str]:
    start = line_index
    paren_balance = 0
    while start > 0 and line_index - start < max_radius:
        previous = lines[start - 1].strip()
        if previous.endswith(";") or previous.endswith("{") or previous.endswith("}"):
            break
        start -= 1

    end = line_index
    while end + 1 < len(lines) and end - line_index < max_radius:
        text = "\n".join(lines[start : end + 1])
        paren_balance = text.count("(") - text.count(")")
        if paren_balance <= 0 and (";" in lines[end] or "{" in lines[end]):
            break
        end += 1

    return start + 1, end + 1, "\n".join(lines[start : end + 1])


def split_parameters(parameter_text: str) -> list[str]:
    params: list[str] = []
    start = 0
    paren_depth = 0
    angle_depth = 0
    bracket_depth = 0

    for index, char in enumerate(parameter_text):
        if char == "(":
            paren_depth += 1
        elif char == ")" and paren_depth > 0:
            paren_depth -= 1
        elif char == "<":
            angle_depth += 1
        elif char == ">" and angle_depth > 0:
            angle_depth -= 1
        elif char == "[":
            bracket_depth += 1
        elif char == "]" and bracket_depth > 0:
            bracket_depth -= 1
        elif char == "," and paren_depth == 0 and angle_depth == 0 and bracket_depth == 0:
            params.append(parameter_text[start:index].strip())
            start = index + 1

    final = parameter_text[start:].strip()
    if final:
        params.append(final)
    if len(params) == 1 and params[0] in {"void", ""}:
        return []
    return params


def extract_signature_parameters(statement: str) -> tuple[str, list[str]] | None:
    compact = re.sub(r"\s+", " ", statement).strip()
    if not compact or compact.startswith("#"):
        return None

    compact = re.sub(r"\)\s*:\s*.*$", ")", compact)

    open_index = compact.find("(")
    close_index = compact.rfind(")")
    if open_index == -1 or close_index == -1 or close_index < open_index:
        return None

    name = extract_function_name(compact[: close_index + 1])
    if not name:
        return None

    return name, split_parameters(compact[open_index + 1 : close_index])


def changed_intersects(selected_lines: set[int] | None, start: int, end: int) -> bool:
    if selected_lines is None:
        return True
    return any(line in selected_lines for line in range(start, end + 1))


def line_is_selected(selected_lines: set[int] | None, line_number: int) -> bool:
    return selected_lines is None or line_number in selected_lines


def severity_for_existing(changed_severity: str, existing_severity: str, selected_lines: set[int] | None) -> str:
    return changed_severity if selected_lines is not None else existing_severity


def allow_rule(raw_lines: list[str], line_number: int, rule: str) -> bool:
    start = max(1, line_number - 2)
    for current in range(start, line_number + 1):
        if current > len(raw_lines):
            continue
        line = raw_lines[current - 1]
        if "architecture-scan: allow" not in line and "arch-scan: allow" not in line:
            continue
        if "allow all" in line or f"allow {rule}" in line:
            return True
    return False


def add_finding(
    findings: list[Finding],
    raw_lines: list[str],
    severity: str,
    rule: str,
    path: Path,
    line: int,
    message: str,
    remediation: str,
) -> None:
    if allow_rule(raw_lines, line, rule):
        return
    findings.append(Finding(severity, rule, path, line, message, remediation))


def nearest_type_scope(context: tuple[ScopeView, ...]) -> ScopeView | None:
    if any(scope.kind == "function" for scope in context):
        return None
    for scope in reversed(context):
        if scope.kind in {"class", "struct"}:
            return scope
    return None


def is_member_declaration(line: str) -> tuple[str, str] | None:
    stripped = line.strip()
    if not stripped.endswith(";"):
        return None
    if "(" in stripped or stripped.startswith("#"):
        return None
    if re.match(r"^(using|typedef|friend|return|static_assert|template|class|struct|enum|namespace)\b", stripped):
        return None
    if re.match(r"^(public|private|protected)\s*:", stripped):
        return None

    if stripped.startswith("static constexpr ") or stripped.startswith("constexpr "):
        return None

    match = re.match(
        r"(?P<type>(?:(?:const|volatile|mutable|unsigned|signed|long|short)\s+)*(?:[A-Za-z_]\w*::)*[A-Za-z_]\w*(?:\s*<[^;{}()]*>)?(?:\s+[*&]?\s*|\s*[*&]\s*))(?P<name>_?[A-Za-z_]\w*)\s*(?:\[[^\]]*\])?\s*(?:=\s*[^;]*)?;",
        stripped,
    )
    if not match:
        return None
    return match.group("type").strip(), match.group("name")


def is_global_declaration(line: str) -> bool:
    stripped = line.strip()
    if not stripped.endswith(";") or stripped.startswith("#"):
        return False
    if "(" in stripped:
        return False
    if re.match(r"^(using|typedef|friend|return|static_assert|template|class|struct|enum|namespace)\b", stripped):
        return False
    if re.search(r"\b(constexpr|constinit|consteval)\b", stripped):
        return False
    if stripped.startswith("extern "):
        return False
    return bool(re.search(r"\b[A-Za-z_]\w*\s+_?[A-Za-z_]\w*\s*(?:=|;|\[)", stripped))


def is_public_storage(scope: ScopeView, declaration_type: str) -> bool:
    if "static constexpr" in declaration_type or declaration_type.startswith("constexpr"):
        return False
    return scope.access == "public"


def scan_anonymous_namespaces(
    path: Path,
    raw_lines: list[str],
    clean_lines: list[str],
    selected_lines: set[int] | None,
    findings: list[Finding],
) -> None:
    for index, line in enumerate(clean_lines, start=1):
        direct = re.match(r"^\s*namespace\s*\{", line)
        split = re.match(r"^\s*namespace\s*$", line)
        split_line = index
        if split:
            for lookahead in range(index + 1, min(index + 4, len(clean_lines) + 1)):
                if clean_lines[lookahead - 1].strip():
                    split_line = lookahead
                    split = re.match(r"^\s*\{", clean_lines[lookahead - 1])
                    break
        if not direct and not split:
            continue
        if not line_is_selected(selected_lines, index) and not line_is_selected(selected_lines, split_line):
            continue
        add_finding(
            findings,
            raw_lines,
            "P0",
            "no-anonymous-namespace",
            path,
            index,
            "Anonymous namespace hides ownership in project-owned source.",
            "Move reusable behavior to the authoritative owner or named vocabulary module; move owner-local behavior to private members; replace explanation-only helpers with comments.",
        )


def scan_structs_and_enums(
    path: Path,
    raw_lines: list[str],
    clean_lines: list[str],
    selected_lines: set[int] | None,
    findings: list[Finding],
) -> None:
    visited_spans: set[tuple[int, int]] = set()
    for index, line in enumerate(clean_lines):
        line_number = index + 1
        if not line_is_selected(selected_lines, line_number):
            continue

        start, end, statement = statement_span_for_line(clean_lines, index)
        if (start, end) in visited_spans:
            continue
        visited_spans.add((start, end))
        compact = re.sub(r"\s+", " ", statement).strip()

        struct_match = re.search(r"\bstruct\s+([A-Za-z_]\w*)\b", compact)
        if struct_match and not re.search(r"\b(friend|enum)\s+struct\b", compact):
            name = struct_match.group(1)
            is_definition = "{" in compact
            severity = severity_for_existing("P0" if is_definition else "P1", "P2", selected_lines)
            rule = "new-struct-type" if is_definition else "struct-forward-declaration"
            add_finding(
                findings,
                raw_lines,
                severity,
                rule,
                path,
                line_number,
                f"`struct {name}` is presumed to be a transport bag or hidden ownership split.",
                "Use an encapsulated domain vocabulary type with complete operations, or move the state/behavior into the authoritative owner so callers do not transport it.",
            )
            if name.endswith(SUSPICIOUS_TYPE_SUFFIXES):
                add_finding(
                    findings,
                    raw_lines,
                    severity_for_existing("P1", "P2", selected_lines),
                    "suspicious-type-name",
                    path,
                    line_number,
                    f"`{name}` has a suffix associated with transport, helper, or wrapper drift.",
                    "Confirm this is intrinsic domain vocabulary; otherwise move behavior to the canonical owner and delete the support type.",
                )

        class_match = re.search(r"\bclass\s+(?:EXPORT\s+)?([A-Za-z_]\w*)\b", compact)
        if class_match:
            name = class_match.group(1)
            if name.endswith(SUSPICIOUS_TYPE_SUFFIXES):
                add_finding(
                    findings,
                    raw_lines,
                    severity_for_existing("P1", "P2", selected_lines),
                    "suspicious-type-name",
                    path,
                    line_number,
                    f"`class {name}` has a suffix associated with transport, helper, or wrapper drift.",
                    "Confirm this class is an authoritative behavioral owner; avoid helper/facade/support layers that mostly forward or move data.",
                )

        enum_match = re.search(r"\benum(?:\s+class)?\s+([A-Za-z_]\w*)\b", compact)
        if enum_match:
            name = enum_match.group(1)
            add_finding(
                findings,
                raw_lines,
                severity_for_existing("P0", "P2", selected_lines),
                "new-simple-classifier",
                path,
                line_number,
                f"`enum {name}` may be a simple classifier rather than domain algebra.",
                "Prefer existing domain vocabulary or a complete algebraic vocabulary type with companion operations and tests; do not use classifiers to steer another owner's internals.",
            )


def scan_storage_declarations(
    path: Path,
    raw_lines: list[str],
    clean_lines: list[str],
    contexts: list[tuple[ScopeView, ...]],
    selected_lines: set[int] | None,
    findings: list[Finding],
) -> None:
    for line_number, line in enumerate(clean_lines, start=1):
        if not line_is_selected(selected_lines, line_number):
            continue

        context = contexts[line_number] if line_number < len(contexts) else tuple()
        member = is_member_declaration(line)
        if member:
            declaration_type, name = member
            scope = nearest_type_scope(context)
            if scope is not None:
                if re.search(r"\bbool\b", declaration_type):
                    add_finding(
                        findings,
                        raw_lines,
                        severity_for_existing("P0", "P2", selected_lines),
                        "new-bool-member",
                        path,
                        line_number,
                        f"`{name}` is a bool data member in `{scope.name}`.",
                        "Use dense internal representation, derive the fact, or expose domain behavior. A rich API with compact storage is the project standard.",
                    )

                if is_public_storage(scope, declaration_type):
                    add_finding(
                        findings,
                        raw_lines,
                        severity_for_existing("P0", "P2", selected_lines),
                        "public-storage",
                        path,
                        line_number,
                        f"`{name}` is public storage on `{scope.name}`.",
                        "Keep representation private and expose operations. Public fields make dense layout and invariant enforcement harder.",
                    )

                if re.search(r"\b(double|size_t|long|int)\b", declaration_type) and not re.search(
                    r"\b(uint\d+_t|int\d+_t)\b", declaration_type
                ):
                    add_finding(
                        findings,
                        raw_lines,
                        severity_for_existing("P1", "P2", selected_lines),
                        "wide-storage-member",
                        path,
                        line_number,
                        f"`{name}` uses a wide or platform-sized storage type.",
                        "Use intentionally sized representation for storage-sensitive domain/runtime types, or justify why the owner needs this width.",
                    )
            continue

        if any(scope.kind == "function" for scope in context):
            continue

        if is_global_declaration(line):
            add_finding(
                findings,
                raw_lines,
                severity_for_existing("P0", "P2", selected_lines),
                "file-scope-mutable-state",
                path,
                line_number,
                "Mutable file-scope or namespace-scope state was introduced.",
                "Move state into the authoritative statically allocated owner. File-scope storage is an ownership shortcut.",
            )


def scan_allocations_and_singletons(
    path: Path,
    raw_lines: list[str],
    clean_lines: list[str],
    selected_lines: set[int] | None,
    findings: list[Finding],
) -> None:
    is_production = is_production_path(path)
    key = path_key(path)

    allocation_pattern = re.compile(
        r"\b(?:new|malloc|calloc|realloc)\s*(?:<|\(|[A-Za-z_:])|::new\s*\(|\bmake_unique\s*<|\bmake_shared\s*<"
    )
    for line_number, line in enumerate(clean_lines, start=1):
        if not line_is_selected(selected_lines, line_number):
            continue

        if allocation_pattern.search(line):
            severity = "P0" if is_production else "P2"
            add_finding(
                findings,
                raw_lines,
                severity_for_existing(severity, "P2", selected_lines),
                "new-dynamic-allocation",
                path,
                line_number,
                "Dynamic allocation appears in source.",
                "Production runtime should use static ownership. Move storage into the canonical owner or document an explicit exception.",
            )


def scan_boundary_and_logging(
    path: Path,
    raw_lines: list[str],
    clean_lines: list[str],
    selected_lines: set[int] | None,
    findings: list[Finding],
) -> None:
    key = path_key(path)
    platform_tokens = re.compile(r"\b(_WIN32|_WIN64|_MSC_VER|ARDUINO|CORE_TEENSY|ARDUINO_TEENSY\d*|TEENSY|__arm__|__IMXRT\d*__)\b")
    preprocessor = re.compile(r"^\s*#\s*(if|ifdef|ifndef|elif)\b")
    raw_logging = re.compile(r"\blogging\.txt\b|\bstd::ofstream\b|\bMmLogLogger\s+[A-Za-z_]\w*|\bSD\.open\s*\(")

    for line_number, line in enumerate(clean_lines, start=1):
        if not line_is_selected(selected_lines, line_number):
            continue
        raw_line = raw_lines[line_number - 1] if line_number <= len(raw_lines) else ""

        if key not in BOUNDARY_FILES and preprocessor.search(line) and platform_tokens.search(line):
            add_finding(
                findings,
                raw_lines,
                severity_for_existing("P0", "P2", selected_lines),
                "platform-boundary-drift",
                path,
                line_number,
                "Host/Teensy or platform conditional logic appears outside the centralized boundary.",
                "Move platform redirection into Defines.h or the designated boundary instead of scattering local shims.",
            )

        if is_production_path(path) and key not in RAW_LOGGING_OWNER_FILES and raw_logging.search(raw_line):
            add_finding(
                findings,
                raw_lines,
                severity_for_existing("P0", "P2", selected_lines),
                "raw-logging-ownership",
                path,
                line_number,
                "Production code appears to own or directly open logging infrastructure.",
                "Use the SharedRobotRuntime-owned text log or MmLogLogger instance. Modes should not create parallel logging owners.",
            )


def scan_function_signatures(
    path: Path,
    raw_lines: list[str],
    clean_lines: list[str],
    contexts: list[tuple[ScopeView, ...]],
    selected_lines: set[int] | None,
    findings: list[Finding],
) -> None:
    visited_spans: set[tuple[int, int]] = set()

    for index, line in enumerate(clean_lines):
        line_number = index + 1
        if not line_is_selected(selected_lines, line_number):
            continue
        context = contexts[line_number] if line_number < len(contexts) else tuple()
        if any(scope.kind == "function" for scope in context):
            continue
        if "(" not in line:
            continue

        start, end, statement = statement_span_for_line(clean_lines, index)
        if (start, end) in visited_spans:
            continue
        visited_spans.add((start, end))
        parsed = extract_signature_parameters(statement)
        if not parsed:
            continue
        name, params = parsed
        if not changed_intersects(selected_lines, start, end):
            continue

        if len(params) > 4:
            add_finding(
                findings,
                raw_lines,
                severity_for_existing("P1", "P2", selected_lines),
                "wide-interface",
                path,
                start,
                f"`{name}` has {len(params)} parameters.",
                "Wide signatures usually mean caller-transported owner state. Move state and behavior to the authoritative owner.",
            )

        bool_params = [param for param in params if re.search(r"\bbool\b", param)]
        if bool_params:
            add_finding(
                findings,
                raw_lines,
                severity_for_existing("P1", "P2", selected_lines),
                "flag-argument",
                path,
                start,
                f"`{name}` has bool parameter(s): {', '.join(bool_params)}.",
                "A bool parameter often collapses multiple semantic operations. Prefer owner behavior or a richer domain vocabulary.",
            )


def scan_pass_through_methods(
    path: Path,
    raw_lines: list[str],
    clean_source: str,
    selected_lines: set[int] | None,
    findings: list[Finding],
) -> None:
    for function in find_function_definitions(clean_source):
        if not changed_intersects(selected_lines, function.start_line, function.end_line):
            continue
        body_lines = [line.strip() for line in function.body.splitlines() if line.strip() and line.strip() not in {"{", "}"}]
        if len(body_lines) > 2:
            continue
        body = " ".join(body_lines)
        if not re.search(r"(?:return\s+)?[A-Za-z_]\w*(?:\.|->)[A-Za-z_]\w*\s*\(", body):
            continue
        add_finding(
            findings,
            raw_lines,
            severity_for_existing("P1", "P2", selected_lines),
            "pass-through-method",
            path,
            function.start_line,
            f"`{function.name}` appears to mostly forward to another object.",
            "Forwarding surfaces are usually alternate access paths. Call the canonical owner directly or move real behavior into this owner.",
        )


def scan_timing_and_motion(
    path: Path,
    raw_lines: list[str],
    clean_lines: list[str],
    selected_lines: set[int] | None,
    findings: list[Finding],
) -> None:
    timing_pattern = re.compile(r"\bLastDiagnostics\s*\(\s*\)\s*\.\s*dtUs\b")
    wait_pattern = re.compile(r"\b(delay|Sleep)\s*\(|std::this_thread::sleep_|wait one tick", re.IGNORECASE)
    boot_pin_pattern = re.compile(r"\bPinPair\s*\(|\bIsPinPairStrapMonitorClosed\s*\(")
    maneuver_geometry_pattern = re.compile(r"\bSmoothTurn|TurnProfile|ManeuverGeometry|turnRadius|arcRadius")

    for line_number, line in enumerate(clean_lines, start=1):
        if not line_is_selected(selected_lines, line_number):
            continue

        if timing_pattern.search(line):
            add_finding(
                findings,
                raw_lines,
                severity_for_existing("P1", "P2", selected_lines),
                "completed-tick-cadence",
                path,
                line_number,
                "`LastDiagnostics().dtUs` is being read.",
                "Completed-tick diagnostics must not provide live active-control cadence. Use the current tick/session timing owner.",
            )

        if path_key(path) not in BOUNDARY_FILES and wait_pattern.search(line):
            add_finding(
                findings,
                raw_lines,
                severity_for_existing("P1", "P2", selected_lines),
                "blocking-or-sleeping-control-flow",
                path,
                line_number,
                "Blocking wait, sleep, or delay appears outside the centralized boundary.",
                "Mode/control flow should be callback-driven. Pause callbacks may block only while the robot is stationary.",
            )

        if "BootModeRegistry" not in path.name and boot_pin_pattern.search(line):
            add_finding(
                findings,
                raw_lines,
                severity_for_existing("P1", "P2", selected_lines),
                "boot-selector-drift",
                path,
                line_number,
                "Boot selector pin or strap logic appears outside BootModeRegistry.",
                "Keep top-level boot selection metadata in BootModeRegistry or the platform pin map.",
            )

        if is_production_path(path) and maneuver_geometry_pattern.search(line) and "Maneuver" not in path.name:
            add_finding(
                findings,
                raw_lines,
                severity_for_existing("P1", "P2", selected_lines),
                "maneuver-vocabulary-drift",
                path,
                line_number,
                "Motion geometry vocabulary appears outside the maneuver owners.",
                "Derive maneuver facts from ManeuverSet, ManeuverCode, ManeuverInstance, and Maze::GetCellDimension() instead of local geometry bags.",
            )


def scan_test_assertions(
    path: Path,
    raw_lines: list[str],
    clean_lines: list[str],
    selected_lines: set[int] | None,
    findings: list[Finding],
) -> None:
    if not is_test_path(path):
        return

    for line_number, line in enumerate(clean_lines, start=1):
        if not line_is_selected(selected_lines, line_number):
            continue
        raw = raw_lines[line_number - 1] if line_number <= len(raw_lines) else ""
        if "isfinite" in line or "IsFinite" in line:
            add_finding(
                findings,
                raw_lines,
                severity_for_existing("P1", "P2", selected_lines),
                "weak-test-assertion",
                path,
                line_number,
                "Test assertion appears to check only finite output.",
                "If this test protects an owner/path contract, assert that contract directly rather than accepting a smoke result.",
            )
        if re.search(r"\bbatteryVoltageV\s*=\s*0(?:\.0f?)?\b", raw):
            add_finding(
                findings,
                raw_lines,
                severity_for_existing("P1", "P2", selected_lines),
                "impossible-test-default",
                path,
                line_number,
                "Test setup appears to use zero battery voltage.",
                "Avoid defaults that normalize impossible runtime states. Use production-like test inputs unless the impossible state is the point.",
            )


def scan_file(root: Path, path: Path, selected_lines: set[int] | None) -> list[Finding]:
    full_path = root / path
    try:
        source = full_path.read_text(encoding="utf-8", errors="replace")
    except OSError:
        return []

    raw_lines = source.splitlines()
    clean_source = strip_cpp_comments_and_strings(source)
    clean_lines = clean_source.splitlines()
    contexts = build_scope_contexts(clean_source)
    findings: list[Finding] = []

    scan_anonymous_namespaces(path, raw_lines, clean_lines, selected_lines, findings)
    scan_structs_and_enums(path, raw_lines, clean_lines, selected_lines, findings)
    scan_storage_declarations(path, raw_lines, clean_lines, contexts, selected_lines, findings)
    scan_allocations_and_singletons(path, raw_lines, clean_lines, selected_lines, findings)
    scan_boundary_and_logging(path, raw_lines, clean_lines, selected_lines, findings)
    scan_function_signatures(path, raw_lines, clean_lines, contexts, selected_lines, findings)
    scan_pass_through_methods(path, raw_lines, clean_source, selected_lines, findings)
    scan_timing_and_motion(path, raw_lines, clean_lines, selected_lines, findings)
    scan_test_assertions(path, raw_lines, clean_lines, selected_lines, findings)

    return findings


def sorted_findings(findings: Iterable[Finding]) -> list[Finding]:
    return sorted(
        findings,
        key=lambda finding: (
            SEVERITY_ORDER[finding.severity],
            finding.path.as_posix().lower(),
            finding.line,
            finding.rule,
        ),
    )


def finding_to_dict(root: Path, finding: Finding) -> dict[str, object]:
    return {
        "severity": finding.severity,
        "rule": finding.rule,
        "path": finding.path.as_posix(),
        "absolute_path": str((root / finding.path).resolve()),
        "line": finding.line,
        "message": finding.message,
        "remediation": finding.remediation,
    }


def print_text_report(root: Path, findings: list[Finding], scanned_files: list[Path], max_findings: int) -> None:
    counts: dict[str, int] = {"P0": 0, "P1": 0, "P2": 0}
    for finding in findings:
        counts[finding.severity] += 1

    print("Architecture smell scan")
    print(f"Files scanned: {len(scanned_files)}")
    print(f"Findings: P0={counts['P0']} P1={counts['P1']} P2={counts['P2']} total={len(findings)}")

    displayed = findings if max_findings <= 0 else findings[:max_findings]
    if max_findings > 0 and len(findings) > max_findings:
        print(f"Showing first {max_findings} findings. Re-run with --max-findings 0 for all findings.")

    for finding in displayed:
        absolute = (root / finding.path).resolve()
        print()
        print(f"{finding.severity} {finding.rule}")
        print(f"  {finding.path.as_posix()}:{finding.line}")
        print(f"  {finding.message}")
        print(f"  {finding.remediation}")
        print(f"  file: {absolute}")


def should_fail(findings: list[Finding], fail_on: str) -> bool:
    if fail_on == "none":
        return False
    threshold = SEVERITY_ORDER[fail_on]
    return any(SEVERITY_ORDER[finding.severity] <= threshold for finding in findings)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Repo-specific architecture tripwire for MicroMouse cleanup work."
    )
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--full", action="store_true", help="Scan all tracked project-owned C++ source files.")
    mode.add_argument("--changed", action="store_true", help="Scan added/changed lines relative to --base. Default.")
    mode.add_argument("--staged", action="store_true", help="Scan staged added/changed lines only.")
    parser.add_argument("--base", default="HEAD", help="Git base for --changed. Default: HEAD.")
    parser.add_argument("--root", type=Path, default=Path.cwd(), help="Repository root. Default: current directory.")
    parser.add_argument("--production-only", action="store_true", help="Exclude MazeMapTest from the scan.")
    parser.add_argument(
        "--fail-on",
        choices=["P0", "P1", "P2", "none"],
        default="P0",
        help="Exit non-zero when findings at this severity or higher are present. Default: P0.",
    )
    parser.add_argument("--format", choices=["text", "json"], default="text")
    parser.add_argument("--max-findings", type=int, default=100, help="0 means no limit in text output.")
    args = parser.parse_args()

    root = args.root.resolve()
    include_tests = not args.production_only
    scan_mode = "full" if args.full else "staged" if args.staged else "changed"

    try:
        if scan_mode == "full":
            files = load_tracked_source_files(root, include_tests)
            if args.production_only:
                files = [path for path in files if is_production_path(path)]
            selected_by_file: dict[Path, set[int] | None] = {path: None for path in files}
        else:
            changed = load_changed_lines(root, scan_mode, args.base, include_tests)
            if args.production_only:
                changed = {path: lines for path, lines in changed.items() if is_production_path(path)}
            files = sorted(path for path in changed if (root / path).exists())
            selected_by_file = {path: changed[path] for path in files}
    except subprocess.CalledProcessError as error:
        print(f"architecture_smell_scan: git command failed: {error}", file=sys.stderr)
        return 2

    findings: list[Finding] = []
    for path in files:
        findings.extend(scan_file(root, path, selected_by_file[path]))

    findings = sorted_findings(findings)

    if args.format == "json":
        payload = {
            "mode": scan_mode,
            "files_scanned": [path.as_posix() for path in files],
            "findings": [finding_to_dict(root, finding) for finding in findings],
        }
        print(json.dumps(payload, indent=2))
    else:
        print_text_report(root, findings, files, args.max_findings)

    return 1 if should_fail(findings, args.fail_on) else 0


if __name__ == "__main__":
    raise SystemExit(main())
