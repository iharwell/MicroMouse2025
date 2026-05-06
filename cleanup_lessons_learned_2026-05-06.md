# Cleanup Lessons Learned Log

Date: 2026-05-06

Primary context:
- `VehicleState.h` convergence cleanup
- follow-on repair commit `b309b77785b3a482d88c33b4e2e33fb92ca87930`

## Purpose

This note is for future contributors, including agentic contributors, who need to learn from this cleanup failure without having to rediscover the same problems the hard way.

It records:

- the reasoning patterns that led to poor decisions
- why those decisions were architecturally wrong
- what damage they created or propagated
- what replacement heuristics should be used next time

This is not a blame note. It is a design-failure extraction log. The point is to squeeze as much reusable understanding as possible out of one bad episode so that later work starts from better instincts.

## How to read this note

Read each lesson as a pattern, not as a one-off historical anecdote.

For each section, ask:

1. am I currently using the same line of reasoning?
2. am I accepting a similar tradeoff because it feels locally convenient?
3. if I made this mistake again, what visible fallout would appear first?

The details below are tied to one specific cleanup, but the underlying mistakes are common:

- mistaking transport shape for architecture
- deleting a smell without deleting the underlying ownership problem
- widening interfaces while believing they are becoming narrower
- using code shape to compensate for missing conceptual clarity

The practical goal is not merely “avoid this exact bug.” The practical goal is to improve judgment. A failed cleanup should pay for future accuracy.

## External Grounding

The lessons here are consistent with several widely respected sources:

- David Parnas, *On the Criteria to Be Used in Decomposing Systems into Modules*.
  - Core idea: decompose around hidden design decisions, not around steps in execution.
  - CMU copy: [kilthub.cmu.edu/articles/journal_contribution/On_the_criteria_to_be_used_in_decomposing_systems_into_modules/6607958](https://kilthub.cmu.edu/articles/journal_contribution/On_the_criteria_to_be_used_in_decomposing_systems_into_modules/6607958)
  - DOI: [10.1145/361598.361623](https://doi.org/10.1145/361598.361623)

- John Ousterhout, *A Philosophy of Software Design*.
  - Core ideas used here:
    - deep modules hide complexity behind small interfaces
    - information leakage and temporal decomposition are red flags
    - pass-through methods and pass-through arguments usually indicate bad responsibility division
    - comments should capture information not obvious from code
  - Accessible PDF: [milkov.tech/assets/psd.pdf](https://milkov.tech/assets/psd.pdf)

- C++ Core Guidelines.
  - Core ideas used here:
    - interfaces are the most important part of code organization
    - keep argument counts low
    - many-argument APIs often indicate a missing abstraction or too many responsibilities
    - encapsulate messy constructs instead of spreading them through the code
    - minimize exposure
  - Official site: [isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines](https://isocpp.github.io/CppCoreGuidelines/CppCoreGuidelines?lang=en)

- Martin Fowler, *Code Smell* and *Flag Argument*.
  - Core ideas used here:
    - a data-only class is often a sign that behavior is in the wrong place
    - flag arguments obscure intent and often indicate that the interface is serving more than one semantic operation
  - [martinfowler.com/bliki/CodeSmell.html](https://martinfowler.com/bliki/CodeSmell.html)
  - [martinfowler.com/bliki/FlagArgument.html](https://martinfowler.com/bliki/FlagArgument.html)

These sources do not dictate the repo’s local rules, but they strongly reinforce them. The repo’s cleanup policy is stricter than mainstream guidance in some places, yet it is stricter in directions that these sources generally support: narrower interfaces, clearer ownership, deeper modules, and less information leakage.

## Lesson 1: “Bag or long parameter list?” was the wrong question

### Bad reasoning

I repeatedly framed design choices as:

- should this be a bag?
- if not, should I pass the pieces individually?
- if not, should I create a context object?

That was a false design space. It assumes the caller should be responsible for assembling the owner’s operating state.

### Why this was attractive

It felt practical:

- bags reduced signature churn
- long parameter lists looked “more explicit”
- contexts looked like a compromise that preserved call-site convenience

This is a common mainstream tradeoff, so it felt familiar and defensible.

### Why it was wrong

In this repo, bags and long parameter lists are usually not rival solutions. They are sibling symptoms. They usually mean that the wrong thing is outside the owner.

The C++ Core Guidelines explicitly say that too many arguments usually mean either a missing abstraction or a function that is doing too many jobs. Ousterhout makes the same point from a different angle: the goal is to hide complexity in a deeper module with a simpler interface, not to force callers to know more. Parnas’s modularization principle goes further: the decomposition should hide design decisions, not externalize them.

So when a refactor pressures the code toward either:

- a bag of fields, or
- many scalar parameters,

the right response is usually not to choose between them. The right response is to ask which owner should retain and interpret that state so the caller does not have to transport it.

### Fallout

This mistake created or encouraged:

- `ModelCycleContext`
- `ControlCycleTiming`
- later getter sprawl when those bags were removed
- widened seams between `PlantModel`, `SrUkfCore`, `DriveBase`, and `LoopController`
- tests that had to know too much about internal data flow

The cleanup ended up deleting one class of bag while preserving the same structural problem in other shapes.

### Better rule

When a design seems to need either a bag or a long parameter list, stop and ask:

- what owner should hold this state?
- what behavior belongs with it?
- how should the interface change so that callers stop carrying this knowledge around?

If that question is not answered, the refactor is not ready.

## Lesson 2: Deleting a bag without converging ownership just creates shrapnel

### Bad reasoning

I treated “remove the bag” as a sufficient cleanup objective. Once the bag was gone, I counted the area as converged even if the same knowledge resurfaced as:

- getters
- wrappers
- forwarding methods
- helper-specific local vocabularies
- duplicated reconstruction code

### Why this was attractive

It produced visible local progress:

- headers got smaller
- certain types disappeared
- call sites compiled

That looked like cleanup momentum.

### Why it was wrong

Ousterhout’s warning about pass-through methods and pass-through arguments applies directly here. A bad abstraction does not become good just because it is distributed across more methods and types. In fact, once the same knowledge is split across more entry points, the information leakage often gets worse.

Parnas’s point about hidden design decisions matters here too: if multiple modules or APIs still have to know the same design decision, then the system has not actually hidden it.

Deleting a bag is only a win if the underlying knowledge:

- moved into one canonical owner, or
- disappeared because it was truly dead

If the knowledge remains and is still visible everywhere, then the bag was only one symptom.

### Fallout

This mistake directly led to:

- `SrUkfCore` getter sprawl replacing `modelCycleContext()`
- duplicated telemetry assembly in `DriveBase`
- split command/reporting authority between `PlantModel` and `SrUkfCore`
- split command-reporting reads between `DriveTelemetry` and direct `DriveBase` getters

### Better rule

Do not measure cleanup progress by “types deleted.” Measure it by:

- number of ownership paths removed
- number of call sites that no longer need to know internal details
- whether one owner now fully and behaviorally answers the use case

## Lesson 3: Temporal decomposition was repeatedly mistaken for architecture

### Bad reasoning

I repeatedly let execution order shape APIs and types:

- current tick vs completed tick vs reporting state got blurred together
- plant/UKF runtime state was organized around the cycle pipeline rather than around stable owners
- helper signatures often reflected “the step I am in” instead of “the knowledge needed here”

### Why this was attractive

Control-loop code naturally makes execution order very salient. It is easy to think in terms of:

- start of tick
- read sensors
- predict
- update
- log
- finalize

That made it easy to rationalize “cycle context” and mixed timing surfaces.

### Why it was wrong

This is exactly the trap Ousterhout calls temporal decomposition: structuring modules around the order in which things happen instead of around hidden knowledge. He is explicit that runtime order matters, but it should not control module structure unless that also serves information hiding.

The repo’s `Drive` example demonstrates the better pattern. `Drive` is stateful across time, but its interface is not a bag of tick-stage plumbing. It owns the state that spans time and exposes a narrow semantic surface.

The right question is not “what phase are we in?” The right question is “what owner should retain the knowledge that persists across those phases?”

### Fallout

This mistake directly contributed to:

- `LastDiagnostics()` being pressed into service for live execution cadence
- `ControlCycleTiming` mixing in-progress and completed semantics
- cycle-state knowledge leaking into multiple layers
- call sites that had to understand timing lifecycle details instead of just their own job

### Better rule

When a proposed type or API name is dominated by time order, phase order, or pipeline order, stop and ask:

- is this really an owner, or just a snapshot of the current choreography?

If it is choreography, it probably does not belong as a public or shared architectural concept.

## Lesson 4: I confused “fewer names” with “narrower interface”

### Bad reasoning

I kept collapsing surfaces under the idea that fewer named APIs meant more convergence, even when the result mixed distinct responsibilities.

The clearest example was trying to make `LastDiagnostics()` serve both:

- a completed diagnostic record
- live callback-time cadence

### Why this was attractive

The repo rightly values narrow interfaces. That made “remove accessors” and “merge surfaces” feel directionally correct.

### Why it was wrong

A narrow interface is not just an interface with fewer functions. It is an interface with fewer responsibilities.

The C++ Core Guidelines emphasize precise interfaces, not merely small ones. Ousterhout’s “different layer, different abstraction” rule makes the same distinction: a small surface is not good if it collapses multiple abstractions into one confusing one.

The correct version of narrowness is:

- one capability, expressed clearly
- with the owner retaining the state and complexity

The incorrect version is:

- fewer names
- but more semantic overload per name

### Fallout

This mistake produced:

- stale `dt` usage in active control logic
- a broken documented contract for `LastDiagnostics()`
- confused timing ownership across controllers

### Better rule

For every public method, write down:

- what capability it offers
- what state it assumes
- whether two different callers would expect the same meaning from the result

If two distinct abstractions are being hidden behind one name, the interface is not narrow. It is muddy.

## Lesson 5: I kept designing “transport surfaces” instead of behavioral owners

### Bad reasoning

I repeatedly created or tolerated surfaces whose main job was to move values between layers:

- data-only helper types
- split getter surfaces
- wrapper methods that simply forwarded to another owner with slightly different naming

### Why this was attractive

This kind of code feels innocuous:

- it is easy to write
- it can look explicit
- it avoids editing a larger owner immediately

### Why it was wrong

Fowler’s code-smell discussion is useful here: data-only classes are often a sign that behavior is misplaced. Ousterhout is sharper: pass-through methods usually indicate that the responsibility split is wrong, and each new piece of infrastructure must remove more complexity than it adds.

The repo’s own strongest exemplar is `Drive`. `Drive` is allowed not because it sits in front of `DriveBase`, but because it is a real behavioral owner:

- it retains state
- it captures progression semantics
- it is complete across the justified simplification space
- it makes parallel wrappers obviously invalid

My cleanup often produced the opposite shape:

- more plumbing
- less ownership
- more places where a sibling wrapper could plausibly arise

### Fallout

This mistake contributed to:

- `SrUkfCore` becoming a second drive-solver surface
- `Estimator` becoming an escape hatch instead of a clean seam
- `AppliedTorqueEstimate` becoming a public plumbing type
- `DriveBase` reconstructing UKF diagnostics one field at a time

### Better rule

If a new type or method mostly transports data or forwards another owner’s API, stop.

Ask:

- what behavior does this own?
- what state does this retain?
- what complete caller-facing capability does it provide that the lower owner should not expose directly?

If the answer is weak, the code should not exist.

## Lesson 6: Rich existing language must be used semantically, not merely copied into signatures

### Bad reasoning

I sometimes replaced obviously bad local vocabularies with broader existing ones only at the type-signature level, while preserving the old semantics in the implementation.

The `RelativeDirection` migration for side sensors is the clearest example. The signature widened, but the behavior still effectively encoded a left/right side dialect.

### Why this was attractive

It looked like convergence:

- old type removed
- canonical type appears in the signature

### Why it was wrong

A richer vocabulary is only a win if the behavior truly operates in that language. Otherwise the code is lying about its abstraction.

This repo already has a strong directional language. That language is valuable precisely because it supports composition and indexing semantics across the codebase. A helper that accepts `RelativeDirection` but silently collapses most values onto “right” is not using the language. It is laundering a poorer local concept through a better type name.

### Fallout

This created:

- signatures that advertise richer semantics than the implementation honors
- a latent trap for future callers
- a false sense that local dialect cleanup had finished

### Better rule

When replacing a local vocabulary with an existing canonical one, check three things:

1. the signature uses the canonical type
2. the implementation actually reasons in that language
3. unsupported cases are rejected or narrowed explicitly, not silently coerced

If any of those are false, the migration is incomplete.

## Lesson 7: I added code where a comment would have been better

### Bad reasoning

I often tried to encode rationale, caution, or “clarity” into extra helpers and extra API structure instead of writing concise comments and leaving the code direct.

### Why this was attractive

“Make it obvious in code” is generally good advice, so extra structure can masquerade as rigor.

### Why it was wrong

Ousterhout’s comments guidance is the needed counterweight: comments should capture what is not obvious from the code, including rationale, invariants, and rules. Not every important idea belongs in executable structure.

When the only thing being added is explanation, extra code is often the wrong vehicle because:

- it adds lines
- it creates names that compete for attention
- it can mislead future readers into believing the new structure carries real behavior

The repo’s navigation cost matters here. Lines are not free. A helper that exists mostly to document intent is often worse than a short precise comment above direct owner-local code.

### Fallout

This tendency inflated:

- helper count
- accessor count
- coordination code
- scrolling and visual noise around business logic

### Better rule

If the new code is primarily serving explanation rather than behavior or ownership, prefer a comment.

Write code for behavior. Write comments for rationale, invariants, domain meaning, and cross-cutting rules.

## Lesson 8: I tolerated tests that preserved compilation while weakening architecture coverage

### Bad reasoning

When refactors broke tests that used old owner shapes, I sometimes accepted weaker assertions as a way to keep the tests aligned with the new code path:

- finite outputs
- numeric equivalence
- smoke checks on scattered getters

### Why this was attractive

It feels safer in the moment:

- fewer brittle tests
- easier migration
- less need to rebuild an architectural assertion after a refactor

### Why it was wrong

In this repo, tests do not just protect numerical behavior. They also protect architectural behavior:

- which owner is authoritative
- which path is used
- which fallback path is not used
- which state propagation contract is being exercised

Weakening those assertions makes it easier for structural regressions to pass silently.

This maps directly to Fowler’s notion of smell as a surface clue to a deeper problem. Once a test loses its ability to detect the deeper problem, the smell remains but the alarm is gone.

### Fallout

This created:

- tests that no longer enforced owner/path semantics
- zero-supply and wrong-fan-duty test paths that no longer resembled runtime
- less confidence that “cleanup” preserved the intended architecture

### Better rule

When a refactor changes an owner boundary, ask first:

- what architectural contract was this test protecting?

Preserve that contract, even if the assertion mechanism changes. Never settle for “still finite” if the old test was proving a stronger ownership or path property.

## Lesson 9: I treated “compile-clean” as a stronger signal than it is

### Bad reasoning

I let successful integration and passing builds carry too much weight in judging convergence.

### Why this was attractive

Compilation gives fast feedback. It is easy to mistake “the code graph closes again” for “the architecture is now cleaner.”

### Why it was wrong

Parnas, Ousterhout, Fowler, and the C++ Core Guidelines all converge on the same point from different angles: structurally bad code often compiles cleanly. A smell is not a compiler error. A shallow interface is not a linker error. Information leakage is not a unit-test failure unless the tests are specifically designed to catch it.

This repo is aggressive precisely because “working junk is still junk” is true in practice.

### Fallout

This mindset allowed:

- partial convergence to look complete
- alternate access paths to survive
- tests to lose architectural specificity without immediate alarm

### Better rule

Treat successful build/test results as necessary but weak evidence.

The stronger questions are:

- did the change eliminate an ownership path?
- did it reduce what callers must know?
- did it make a sanctioned simplification layer more complete and singular?
- did it remove or merely relocate the smell?

## Lesson 10: The repo’s strict local rules are not arbitrary style; they are guardrails against recurring failure modes

### Bad reasoning

At times I treated local repo rules as unusually strict style preferences that could be “interpreted intelligently” in edge cases.

### Why this was attractive

Many repos are tolerant of:

- contexts
- helper layers
- local enums
- private transport structs
- staged migrations

That makes exceptions feel normal.

### Why it was wrong

The rules in this repo are strict because the codebase has already paid the price for those tolerances. They are not ornamental. They are targeted countermeasures.

Examples from this cleanup:

- “no bags” was aimed at the exact failure mode that produced `ModelCycleContext`
- “no wrappers/adapters” was aimed at the exact failure mode that produced UKF forwarding seams
- “existing vocabulary first” was aimed at the exact failure mode that produced local directional dialects
- “lines are not free” was aimed at the exact failure mode that produced helper/accessor sprawl

### Fallout

When these rules were treated as negotiable, the refactor drifted back toward familiar industry defaults instead of toward the repo’s intended architecture.

### Better rule

Assume each harsh local rule exists because this codebase has already been harmed by the corresponding “reasonable exception.”

If a design seems to require breaking one of those rules, the default interpretation should be:

- the structure is still wrong

not:

- this is probably one of the harmless exceptions

## What I wish I had going into the work

These are the concise checks that would have prevented most of the damage:

1. If a cleanup seems to force a choice between a bag and a long parameter list, the structure is wrong. Stop and find the owner.
2. If deleting a type causes the same knowledge to reappear as getters, wrappers, or reconstruction code, nothing important was actually removed.
3. If a proposed simplification layer is not as coherent, concise, complete, and statefully exclusive as `Drive`, it probably should not exist.
4. If a new type has little behavior, it is probably plumbing drift rather than a real concept.
5. If a function or helper mostly forwards, mostly re-labels, or mostly assembles data for another owner, it is probably a structural smell.
6. If execution order is driving module structure, check for temporal decomposition.
7. If a richer existing vocabulary is available, using a poorer local one is probably a regression.
8. If a comment can carry the missing information, do not spend code shape on explanation.
9. If a test used to prove an ownership path and now only proves a finite result, the test got worse.
10. If it is hard to name the concept cleanly, the design may still be wrong. Ousterhout explicitly calls “hard to pick name” a red flag.

## Practical operating rules for future cleanup work

- Before coding, produce an owner map:
  - each concept
  - deleted or survives
  - final owner

- For every public API change, answer:
  - what capability is being offered?
  - what state should the owner retain so callers do not transport it?
  - what would the `Drive` version of this interface look like?

- If a proposed change introduces:
  - a new `struct`
  - a new `enum class`
  - a free function
  - an anonymous-namespace helper
  - a context object
  - a forwarding method
  - a direct field-assembly reporting path
  
  stop and justify it explicitly before writing more code.

- Prefer a short authoritative comment over a helper whose main purpose is explanation.

- Evaluate success by:
  - fewer ownership paths
  - less information leakage
  - deeper modules
  - less caller knowledge
  - stronger architectural tests

not by:

- fewer files
- fewer type names
- smaller headers
- passing compilation alone

## Closing summary

The central failure in this cleanup was not “a few bad types survived too long.” The central failure was repeatedly trying to rearrange visible code shape without first forcing the design decision back into one authoritative owner.

Parnas would call that a failure to modularize around hidden design decisions.
Ousterhout would call it a mixture of shallow modules, temporal decomposition, pass-through infrastructure, and information leakage.
Fowler would recognize several of the results as smells that point to deeper structural problems.
The C++ Core Guidelines would describe the public result as imprecise and overexposed interfaces.

The durable lesson is simple:

When architecture is wrong, do not ask which transport shape is least bad.
Fix the owner so the transport shape stops being needed.

If a cleanup goes badly, the right response is not to hide the mistake behind quick repairs or shallow summaries. The right response is to extract every durable lesson possible from the failure and convert that pain into sharper design instincts, stricter review checks, and clearer owner boundaries for the next person.
