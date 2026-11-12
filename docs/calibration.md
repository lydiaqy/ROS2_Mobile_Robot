# Differential-Drive Odometry Calibration

## Problem Identification

During Gazebo simulation, I identified a systematic discrepancy between wheel-odometry estimates and the robot's simulated ground-truth orientation.

When commanding approximately 90 degrees of rotation, the odometry-based controller reported about 89 degrees, while the Gazebo model rotated approximately 101 degrees.

## Calibration Approach

I investigated the differential-drive kinematic parameters and empirically adjusted the `wheel_separation` parameter while keeping the physical wheel positions unchanged.

The robot's geometric wheel separation is 0.34 m. I found that using an effective wheel-separation parameter of 0.304 m improved agreement between the commanded rotation and the simulated ground-truth orientation.

## Experimental Results

| Measurement | Before Calibration | After Calibration |
|---|---|---|
| Geometric wheel separation | 0.34 m | 0.34 m |
| DiffDrive wheel-separation parameter | 0.34 m | 0.304 m |
| Target rotation | 90° | 90° |
| Gazebo ground-truth rotation | Approximately 100.75° | Approximately 90.09° |

The calibrated parameter is an empirical compensation value rather than a change to the robot's physical geometry.

## Engineering Analysis

Potential causes include wheel-ground contact dynamics, simplified caster modeling, and differences between ideal differential-drive kinematics and the physics simulation.

These hypotheses have not yet been independently validated. Additional controlled experiments would be required to establish the root cause.

## Key Takeaway

This investigation demonstrated the importance of comparing odometry estimates against simulation ground truth instead of relying exclusively on controller feedback.

It also provided practical experience in model validation, systematic debugging, and parameter calibration.

## Remaining Work

- Investigate the discrepancy between the controller's angular stopping threshold and reported rotation values.
- Evaluate calibration across different angular velocities.
- Compare complete trajectories using odometry and Gazebo ground truth.
- Investigate the effects of caster contact and wheel friction.