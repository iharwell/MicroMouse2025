# Project Instructions

- All watchdog timers under 60 seconds that trigger a run failure are banned.
- Prefer recovery over fail-fast behavior.
- When behavior deviates from expectation, log the condition before attempting recovery.
- Assume the robot is configured for very high performance.
- The robot can sustain 16.5 m/s^2 of lateral acceleration when the fan is running at 80%.
- Plan strategies with the high-performance operating envelope in mind.
- Follow the guidance and strategy style of the Decimus 5a project.
