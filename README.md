# Character Animation Plugin - İlker

## Build Instructions
1. Open x64 Native Tools Command Prompt for VS 2022
2. `cmake -S . -B build`
3. `cmake --build build --config Release`
4. Copy the generated DLL to `bin/plugins/character/`

## Selected Motions
1. walk_forward
2. push_two_handed
3. climb_low_step

## Features
- Procedural kinematic generation for Walk, Crouch, Jump, and Wave phases.
- Physics-like low pass filter / PD proxy for joint target tracking.
- Controlled exactly 10 major joints as requested.
- Implemented purely via the C API interface, no STL in the ABI.
