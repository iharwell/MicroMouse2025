# Drive Contract Worklog

Date: 2026-04-30

Task:
Swap the revised `Drive.h` documentation into the authoritative repo location and converge `Drive.cpp` and any conflicting tests to the documented contract.

Current semantic decisions:

Overarching guidelines:

1. Preserve the caller's most explicit intent first.
Direct request fields beat inferred meaning.

2. Recover locally, not globally.
If one numeric term is unusable, reinterpret or neutralize that term instead of rejecting the whole instruction.

3. Keep the latest instruction live until explicit supersede or cancel.
Completion is an observation, not implicit lifecycle control.

4. Use signed limits as meaning, not noise.
Their magnitudes cap commands, and their signs help resolve ambiguous direction, but they do not casually override a clearer direct request.

5. For target-defined primitives, the target stays authoritative.
Limits shape the proposal; they do not silently replace a clear "go there / turn there" request with a different target unless the request itself became ambiguous.

6. Live configuration stays live; latched requests stay latched.

7. Semantic recovery happens before degraded attempted motion.
If a coherent interpretation of the latest instruction can still be formed from the primitive semantics, sibling fields, retained live configuration, captured start state, current runtime state, vehicle facts, maze facts, and project conventions, Drive should choose one and try to execute it.

8. Ambiguity is resolved, not treated as failure.
If more than one coherent reading remains possible, Drive should pick one by ordered precedence rather than halting merely because the interpretation is not unique.

9. Degraded attempted motion happens only after semantic recovery has been exhausted.
If the objective or shaping terms cannot be recovered exactly, Drive may neutralize only the failed terms and still attempt motion with the remaining coherent objective.

10. Neutral/brake output is the last resort.
Drive should return neutral/brake-oriented proposals only after it cannot recover either a coherent motion objective or any viable way to pursue that objective. Even then, the installed instruction remains the installed instruction; Drive does not implicitly relabel it as a different primitive.

1. `Start...` members will be implemented as total instruction setters.
That means each `Start...` call replaces the previous instruction instead of no-oping because runtime state, `DriveBase`, or specific numeric arguments are inconvenient or malformed.

2. `MC_NONE` will be treated as a latched no-motion maneuver instruction.
The working meaning is "latest request is a maneuver that currently implies no travel and no turn," so `GetNextControls(...)` should coherently degrade to brake/neutral behavior instead of rejecting the start.

3. `done` will be implemented as a per-tick completion observation, not an implicit cancel.
Completed instructions stay installed until the caller supersedes them with another `Start...` call or explicitly calls `Cancel()`.

4. Malformed scalar request inputs will degrade locally.
If a distance, angle, curvature, speed, or similar scalar cannot be used coherently, the implementation will neutralize only that term and keep the rest of the latest instruction live.

5. Finite negative magnitude-style limits will not be reduced to abs-only semantics.
The current implementation plan is:
- absolute value still supplies the magnitude cap,
- the negative sign remains meaningful as a directional hint when the direct request fields do not already provide a coherent sign,
- if an explicit request sign conflicts with the negative limit sign, the explicit request remains the stronger signal for target-defined primitives and the limit sign remains the fallback signal for ambiguous requests.
Non-finite limit fields remain unavailable. Zero remains a binding zero-cap bound.

6. Live configuration remains live.
`SetLimits(...)`, `SetCommandPDSettings(...)`, and `SetOperationMode(...)` changes must affect later `GetNextControls(...)` calls wherever the active primitive consults those settings.

7. For translation primitives with a coherent progress objective but unusable pace inputs, pace will be recovered from the best remaining local meaning before halting.
Current precedence:
- explicit primitive speed field(s),
- sibling speed field(s) in that primitive,
- captured live motion already in progress when that primitive was armed,
- coherent signed live speed limits,
- only then zero/neutral if no viable pace remains.

Pending checks to keep logging here:

- Whether any remaining malformed-input cases still halt too early instead of recovering a viable interpretation first.
- Whether any existing tests assert the old auto-cancel-on-done behavior or the old start rejection behavior.
- Release-mode verification path and binary freshness check before running tests.
