# Design Document

## Kinematic Approach
The procedural approach computes real-time inverse/forward kinematic targets for Walk, Crouch, Jump, and Wave phases using trigonometric functions bounded to phase timers. 

## PD / Low-Pass Interpolation
To avoid hard snapping and satisfy physics-like continuous motion requirements, the computed `target` quaternions are not directly passed to the bones. Instead, a memory-backed `current_pose` state is maintained. At each tick (dt = 0.02s), the current pose is drawn toward the target pose using a smoothing factor (alpha = 0.25). This naturally mimics a Proportional-Derivative (PD) controller's damping characteristic without the instability of numerical double integration.

## Major Joints Controlled
- ARK_JOINT_UPPERARM_L / R
- ARK_JOINT_LOWERARM_L / R
- ARK_JOINT_THIGH_L / R
- ARK_JOINT_CALF_L / R
- ARK_JOINT_FOOT_L / R
