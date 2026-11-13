# Differential-Drive Odometry Calibration

## Current Configuration

The physical wheel-center separation remains 0.34 m. The DiffDrive plugin uses
the empirical effective separation of 0.304 m again. These are separate Xacro
properties (`geometric_wheel_separation` and `effective_wheel_separation`) so
restoring physical geometry does not accidentally remove odometry compensation.
The 0.34 m plugin setting was used for the transient experiments below; it
reintroduced the known rotation discrepancy and is no longer the default.

The DiffDrive plugin now limits linear acceleration and deceleration to
0.3 m/s² and angular acceleration and deceleration to 0.8 rad/s². The square-path
controller also waits between motions until wheel-odometry linear speed is at
most 0.01 m/s and angular speed is at most 0.02 rad/s continuously for 0.5 s.
Missing or stale odometry resets this settling interval. Final completion and
position-error reporting occur after the last turn has settled.

These changes target starting and stopping transients. Wheel odometry does not
measure body pitch; ground-truth pose is still needed to verify physical stability
and rotation accuracy. The historical calibration results do not establish the
accuracy of the current configuration with acceleration limiting enabled.

`settled odometry rotation` reports accumulated yaw from `/odom` after the
wheel-odometry speeds settle; it does not report Gazebo ground-truth yaw.
The turn controller stops when the angular error is less than 0.02 rad
(about 1.15 degrees), so a settled reading near 89 degrees is consistent with
its stopping threshold. That tolerance does not explain a physical turn near
101 degrees. Acceleration limits and settling waits address transients, while
the effective separation compensates the observed angular scale discrepancy.
Keep both measures and revalidate against ground truth when changing contact
parameters, geometry, or speed.

## Motion-Transient Validation

The historical controlled headless Gazebo tests below used a 0.34 m DiffDrive
wheel-separation parameter, a flat ground plane, the robot geometry described
here, the default physics engine, and a 1 ms physics step. Their runnable scripts
and raw traces are not archived in this repository; these are recorded results,
not measurements newly reproduced after restoring the 0.304 m compensation:

- At a 0.34 m wheel separation, a standalone 0.5 rad/s rotation from rest did not
  produce appreciable body pitch, either with or without acceleration limiting.
- Without acceleration limiting, commanding 0.2 m/s straight motion followed by
  zero velocity and a 0.4 rad/s turn 0.1 s later produced about 1.18° of forward
  pitch. The peak occurred during braking, before the turn started.
- Repeating that command sequence with the current acceleration limits kept
  absolute pitch below 0.001° in this test.
- The updated square-path controller completed four 0.2 m sides, including the
  settling waits. Absolute pitch stayed below 0.001°. The final position closure
  error was about 0.009 m in wheel odometry and 0.087 m in Gazebo ground truth;
  the final ground-truth heading differed from the initial heading by about 44°.

The abrupt-stop test isolates a possible pitching mechanism; it does not
reproduce the square controller's gradual approach to its distance threshold.
These results support the transient mitigation but show that the rotation
calibration discrepancy remains. They do not establish performance at other
speeds, side lengths, or contact settings.

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

These values describe the earlier calibration experiment, not a new test of the
current combination of compensation, acceleration limits, and settling checks.
The calibrated parameter is an empirical compensation value rather than a
change to the robot's physical geometry. Its consistency across speeds and both
turning directions has not been established.

## Engineering Analysis

Potential causes include wheel-ground contact dynamics, simplified caster modeling, and differences between ideal differential-drive kinematics and the physics simulation.

These hypotheses have not yet been independently validated. Additional controlled experiments would be required to establish the root cause.

## Key Takeaway

This investigation demonstrated the importance of comparing odometry estimates against simulation ground truth instead of relying exclusively on controller feedback.

It also provided practical experience in model validation, systematic debugging, and parameter calibration.

## Remaining Work

- Revalidate the restored 0.304 m effective separation with acceleration limiting and settling enabled.
- Evaluate calibration across different angular velocities.
- Compare complete trajectories using odometry and Gazebo ground truth.
- Investigate the effects of caster contact and wheel friction.
