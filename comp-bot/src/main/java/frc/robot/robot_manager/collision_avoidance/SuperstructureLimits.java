package frc.robot.robot_manager.collision_avoidance;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

/** Motion constraints to use for superstructure motion. */
public record SuperstructureLimits(
    TrapezoidProfile.Constraints armConstraints,
    TrapezoidProfile.Constraints elevatorConstraints) {}
