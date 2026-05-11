# Distilled Cleanup Prompt Rules

Source: `cleanup_lessons_learned_2026-05-06.md`

Use these rules as prompt material before architecture cleanup, owner migration, or interface refactoring in this repository.

## Core Rules

Before changing architecture, identify the authoritative owner. If the design pushes data into a bag, context object, long parameter list, getter cluster, or reconstruction code, stop and move the behavior or state back to the owner.

Do not treat deleting a type as cleanup unless the underlying ownership path is also removed. The same knowledge must not reappear as wrappers, getters, forwarding methods, helper vocabularies, or duplicated assembly code.

A new public type, helper, struct, enum, free function, or context object is suspect unless it owns real behavior, retains meaningful state, or represents a functional domain vocabulary with real expressive power.

Do not design transport surfaces. If code mostly forwards, relabels, copies fields, or assembles data for another owner, the responsibility split is probably wrong.

Do not let execution order become architecture. Tick phases, pipeline stages, current-vs-last snapshots, and reporting cadence are choreography unless a real owner retains and interprets that state.

A narrow interface means one clear capability with one clear semantic meaning. Fewer method names are not an improvement if one method now serves multiple concepts.

When adopting domain vocabulary, restrict that label to types that have true functional features enabling expression of domain concepts. `MazeMap/MazeMap/Direction.h` is the key example: it does not merely name directions; it supports absolute and relative direction composition, inversion, turn operations, heading vectors, diagonal checks, and compact directional distance behavior. Do not launder a poorer local dialect through a richer type name unless the implementation actually reasons in that richer vocabulary.

Use comments for rationale, invariants, and domain rules. Do not add helper code whose main purpose is explanation.

When refactoring tests, preserve the architectural contract they protected. Do not weaken an ownership or path assertion into a smoke check, finite-result check, or loose numeric equivalence.

Passing builds and tests are necessary but weak evidence. Judge cleanup by fewer ownership paths, less caller knowledge, less information leakage, and stronger canonical-owner coverage.

Assume strict local repo rules exist because the codebase has already been harmed by the "reasonable exception." If a design seems to require breaking one, assume the structure is still wrong.

Before coding, produce a small owner map: each concept, whether it is deleted or survives, and its final owner. Do not proceed until caller migration and superseded-code deletion are clear.

## Pre-Coding Check

For every proposed cleanup, answer these before editing:

1. Which owner is authoritative for each affected concept?
2. Which wrappers, bags, helpers, getters, or alternate access paths will be deleted?
3. What behavior or state will move inward so callers stop transporting owner knowledge?
4. What tests currently protect owner/path semantics, and how will those assertions survive?
5. What compiled source disappears in the same change so old and new ownership do not coexist?

If these answers are unclear, the refactor is not ready.
