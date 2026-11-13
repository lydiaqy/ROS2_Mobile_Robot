# Mobile Robot Motion Debugging: Resume and Interview Notes

This document explains the engineering work behind a motion-control investigation
and how to present it when applying for Robotics Software Engineer roles in the
Bay Area. Technical results come from the [calibration and validation notes](calibration.md).
Any quantitative resume claim should retain its experimental scope.

Current status: the physical wheel separation is 0.34 m and the DiffDrive
effective separation is 0.304 m. Acceleration limits and odometry-based settling
remain enabled. Restoring the plugin parameter to 0.34 m was an intermediate
comparison, not the final fix. The combined current configuration has not yet
been revalidated against Gazebo ground truth. The physical cause of the rotation
discrepancy remains unconfirmed.

## 1. Career Relevance

For entry-level candidates or career changers, this investigation adds engineering
depth to a ROS 2 mobile robot project. For experienced candidates, it provides a
specific debugging example, while additional experience is needed to demonstrate
physical robot development, complex system design, and deployment skills.

The work demonstrates several relevant skills:

- **System analysis:** Connecting support geometry, center of mass, differential-drive kinematics, and control commands to observed behavior.
- **Experimental design:** Separately testing rotation from rest, abrupt braking, and braking followed by rotation to identify triggering conditions.
- **Control software implementation:** Adding acceleration limits, stopping states, and settling-timer resets after odometry interruptions.
- **Independent validation:** Comparing wheel odometry with Gazebo ground truth to check whether controller feedback represents physical motion.
- **Engineering judgment:** Distinguishing improved pitching behavior from unresolved rotation errors and documenting unverified hypotheses.

These skills are relevant to controls, robotics software integration, simulation,
and validation work. For example, when reviewed during this discussion,
Efference's San Francisco controls software posting explicitly mentioned
diagnosing instability through logs and measurements, testing controlled stops
and communication failures, and validating assumptions in simulation and on
physical robots. This is one job example, rather than a survey of Bay Area hiring.

Reference: [Efference — Robotics Software Engineer, Controls](https://efference.ai/careers/senior-rse-controls/).

## 2. Project Background and Problem

The project uses C++17, ROS 2 Jazzy, Gazebo Harmonic, and URDF/Xacro to simulate a
two-wheel differential-drive robot with odometry-based motion controllers.
A finite-state machine coordinates straight segments and turns for square-path
execution.

During development, the robot was observed pitching forward around turns.
The DiffDrive plugin's wheel-separation parameter had been changed from the
geometric value of 0.34 m to 0.304 m without moving the physical wheels.
The project also showed a discrepancy between wheel-odometry rotation and
Gazebo ground-truth rotation.

The investigation examined when pitching occurred and how the wheel-separation
parameter enters commands and feedback. It did not establish the physical cause
of the rotation discrepancy or prove that 0.304 m is valid across operating
conditions.

## 3. Investigation and Implementation

The model's drive-wheel positions, rear support ball, and center of mass were
examined first. The current robot's overall center of mass is approximately
4.4 cm behind the wheel axle, with its static projection inside the support
region. Abrupt braking after forward motion creates a nose-down tendency.
The rear support ball cannot provide a downward force to hold the rear of the
robot against the ground.

Controlled experiments then measured Gazebo ground-truth orientation directly.
Rotation from rest did not reproduce appreciable pitching. Abrupt braking
followed shortly by rotation did produce pitching, with the peak occurring
before rotation began. This establishes braking transients as one demonstrated
pitching mechanism; it does not establish that they explain every aspect of the
original observation.

DiffDrive converts commands to wheel angular velocities as follows:

```text
right_wheel_speed = (linear_speed + angular_speed * wheel_separation / 2) / wheel_radius
left_wheel_speed  = (linear_speed - angular_speed * wheel_separation / 2) / wheel_radius
```

For rotation in place, reducing the separation parameter from 0.34 m to 0.304 m
reduces the commanded wheel speeds by approximately 10.6% for the same angular
velocity command. The parameter also enters odometry calculations. Changing it
therefore affects both commands and feedback; visual improvement alone does not
establish correct calibration.

The investigation and implementation proceeded as follows:

1. Temporarily restored the plugin wheel separation to the geometric value of 0.34 m for comparison. This removed the earlier empirical compensation and left the rotation discrepancy present.
2. Limited linear acceleration to the range -0.3 to +0.3 m/s² and angular acceleration to -0.8 to +0.8 rad/s² in the DiffDrive plugin, smoothing both acceleration and braking.
3. Added stopping states after straight segments and after turns in the square-path controller.
4. Continued publishing zero-velocity commands until odometry linear speed was at most 0.01 m/s and angular speed was at most 0.02 rad/s continuously for 0.5 seconds.
5. Reset the settling interval when odometry became stale or interrupted, and delayed final completion reporting until the last turn had settled.
6. Restored the effective separation to 0.304 m as a provisional empirical compensation after over-rotation was observed again with the 0.34 m setting. Kept physical separation at 0.34 m and retained acceleration limits and settling checks.

Here, settling is a motion-transition condition based on wheel-odometry speeds.
It does not directly detect body pitch or establish physical stability.
The effective separation is a fitted kinematic parameter, not a demonstrated
physical explanation. Its approximately 10.6% difference from the geometric
value still requires investigation and validation across speeds and directions.

## 4. Validation Results and Limits

The historical transient tests below used the 0.34 m plugin setting, a flat
ground plane, the default physics engine, and a 1 ms physics step. They do not
validate the restored 0.304 m setting combined with acceleration limits and
settling checks. The measurements are retained from earlier investigation
notes; the runnable validation scripts and raw traces are not archived in this
repository, so these figures have not been independently reproduced in the
current review.

| Test | Result |
|---|---|
| Rotation from rest at 0.5 rad/s with a 0.34 m wheel separation | No appreciable forward pitch |
| Forward motion at 0.2 m/s, an abrupt stop, then a 0.4 rad/s turn 0.1 seconds later, without acceleration limits | Peak forward pitch of approximately 1.18°, occurring during braking |
| The same command sequence with acceleration limits of ±0.3 m/s² and ±0.8 rad/s² | Absolute pitch below 0.001° |
| Controller with settling checks executing four 0.2 m sides at the 0.34 m plugin setting | Completed all four turns and settling waits; absolute pitch below 0.001° |
| Settling-logic regression checks | Prevented transitions while measured speed remained above threshold, reset the dwell after an odometry interruption, and resumed after fresh settling samples |

In the complete path test, final position closure error was approximately
0.009 m in wheel odometry and 0.087 m in Gazebo ground truth. The final
ground-truth heading still differed from the initial heading by approximately
44°. The rotation-calibration discrepancy remains unresolved.

The abrupt-stop experiment used a direct stop command. It did not fully
reproduce the square controller's gradual slowdown near its distance threshold.
These measurements come from limited simulation conditions, without statistics
from repeated trials or physical robot validation. The pitch value below 0.001°
describes the simulation readings in these tests only.

## 5. Suggested Resume Wording

Suggested project title:

**ROS 2 Differential-Drive Robot — Motion Control and Simulation Validation**

Technologies: C++17, ROS 2 Jazzy, Gazebo Harmonic, URDF/Xacro, Python.

Choose two of the following bullets, depending on available space:

- Developed a ROS 2/C++ finite-state controller for differential-drive square-path execution, with odometry-based feedback and settling checks between motion segments.
- Implemented acceleration limits and odometry-based stopping states to manage transitions between straight motion and turns.
- Investigated discrepancies between wheel odometry and Gazebo ground-truth rotation, separating empirical wheel-separation compensation from unresolved physical-model causes.

Before using these bullets, be able to explain and reproduce the experiments
independently, and choose verbs that accurately reflect your contribution.
Use numerical pitch-reduction or calibration-accuracy claims only after saving
and reproducing the corresponding experiment with its exact configuration.
The investigation, implementation, and validation used AI coding assistance.
When discussing the development process, accurately explain the tools' role and
how you reviewed the code and evaluated the evidence.

Avoid claims that exceed the current evidence:

- **“Implemented high-precision autonomous navigation.”** Full navigation is not implemented, and ground-truth rotation errors remain unresolved.
- **“Achieved millimeter-level localization accuracy.”** Position closure error is not localization accuracy, and odometry error is not ground-truth error.
- **“Completely solved robot stability.”** The improvement has only been verified under limited simulation conditions.
- **“Identified and fixed the physical cause of over-rotation.”** The 0.304 m setting is a provisional empirical compensation; the root cause remains unconfirmed.
- **“Validated the final configuration.”** Earlier transient tests used the 0.34 m plugin setting; the current combined configuration still needs ground-truth validation.
- **“Built a complete CI regression pipeline.”** The earlier validation scripts and raw traces are not archived in the repository or integrated into continuous testing.

## 6. Sample Interview Explanation

> While developing a ROS 2 differential-drive robot, I observed forward pitching
> around turns and a discrepancy between wheel odometry and Gazebo ground-truth
> rotation. An initial adjustment changed the plugin's wheel separation, but that
> parameter affects both commanded wheel speeds and odometry, so the issues
> needed to be investigated separately.
>
> I examined the center of mass and support geometry, then separated the motion
> into rotation from rest, abrupt braking, and braking followed by rotation.
> Standalone rotation did not produce appreciable pitching, while the braking
> test reached its peak pitch before the turn began. I then added acceleration
> limits and stopping states that wait for measured odometry speeds to settle.
>
> Earlier controlled tests at a 0.34 m plugin setting recorded reduced pitching
> with acceleration limits, but the square-path test still showed accumulated
> ground-truth rotation error. I retained a provisional 0.304 m effective
> separation while keeping the physical wheel spacing at 0.34 m. The rotation
> root cause remains unconfirmed, and the combined configuration still needs
> ground-truth validation. Before citing numerical improvements, I need to
> reproduce and archive the tests.

Be prepared to explain why abrupt braking can cause pitching, why pure rotation
may not produce pitch, how wheel separation enters command and odometry
calculations, how settling handles interrupted feedback, and why `/odom` closure
error alone cannot establish physical motion accuracy.

## 7. Next Steps to Strengthen the Project Evidence

1. Recover or recreate the earlier validation scripts and archive run commands, exact configurations, and raw results in the repository so experiments can be repeated.
2. Save velocity, pitch, and yaw time series before and after the changes, and produce comparison plots or videos.
3. Repeat tests across speeds, acceleration limits, and both turning directions, reporting distributions and maximum errors.
4. Continue investigating the effects of wheel-ground contact, the support ball, and physics parameters on ground-truth rotation.
5. When feasible, validate on physical hardware and document differences between simulation and the real robot.
6. Compare the 0.34 m baseline and 0.304 m compensation with the same acceleration limits and settling checks, recording wheel angles, wheel odometry, and ground-truth pose to test whether a consistent effective separation explains the discrepancy.

For this project, reproducible validation and a stronger explanation of the
rotation discrepancy would make the engineering story easier to substantiate
in an interview.
