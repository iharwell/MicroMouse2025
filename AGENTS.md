# Preferred Project Layout

- Unchanging vehicle facts should be owned by the Vehicle class or composed into members of that class. The Vehicle class serves as the root source of truth for all such facts.
  - These facts should be arranged into composed members that are divided by physical subsystem.
  - Consumers of these facts should only take relevant subsystems as input rather than the entire Vehicle object for ease of testing.
  - Core, base facts should have "_k" prefixes on private fields, "getK" prefixes on accessors. Derived facts should have "d/D" instead of "k/K", and those values should be precalculated and updated rather than evaluated on the fly.
  - Core facts must have a single exclusive owner in this heirarchy, and no external sources independent of this class or it's members.
- The PlantModel class serves as the single source of truth regarding the physical motion model of the robot, including projected movement, calculating required force at the wheels, etc. It should not internalize the parameters of the robot, but should supply the project with shared motion equations.
- The Maze class is the authority on the layout of the maze.
- Locomotion should prefer using the Maneuver primatives when in a maze, 

# Project Organization Guidelines

- Substantial classes should be placed in files sharing the name of that substantive class. Enums and supporting types may be placed in the same file only if they are exclusively consumed by that class.
- Non-template classes should have a .h with the declaration and documentation of members, and a .cpp file with the implementation.
- Project constants should exist as compile-time objects, not top-level constants.
- Software induced limits should be placed in a designated class that is common to all modes and configurations.
- The FloodFill pathfinder should be used for simple navigation, and ManeuverPathfinder should only be used while stationary.
- Structs and classes have the same rules for acceptance: these must offer significant features and behavior, must respect the authority of root truth sources, and should not duplicate behavior. Inheritance should be limited to cases where it offers significant benefit.

# Polished classes

- Use the MazeCell, Maze, MazeLocation, and DirectionalLocation classes as reference for what a good implementation looks like. All new classes should have all public members documented, offer const and non-const options where sensible, feature rich methods, and fully abstract the internal representation from consumers.

# Project Instructions

- No watchdog timers under 60 seconds that trigger a run failure.
  - Prefer recovery over fail-fast behavior.
  - When runtime behavior deviates from expectation, log the condition before attempting recovery.
- The robot can sustain 16.5 m/s^2 of lateral acceleration when the fan is running at 80%.
- Plan strategies with the high-performance operating envelope in mind.
- Use the Decimus 5a project for guidance and reference to understand more about this project.
- Ask clarifying questions before proceeding rather than trying to make-do with initial information.
- Reject any wrapper unless it benefits at least three independent places in code.
- Directional code should respect conventions of +X=right, +Y=forward/up, and +Yaw=clockwise.
- Unit tests should be written first, and not modified without permission.
- Prefer being concise, clear, and writing understandable code to avoiding code churn. A lot of non-conforming code is currently in the codebase that we're trying to clean up, and this requires significant changes.
- Reject structs and classes without members documented, organized headers, or const-aware methods.
- Reject structs and classes with public fields unless there is a clear, documented performance reason for the type to exist.
- Strongly prefer composed types over inheritance except for interface types or where inheritance offers substantial benefits in at least four other places in code.