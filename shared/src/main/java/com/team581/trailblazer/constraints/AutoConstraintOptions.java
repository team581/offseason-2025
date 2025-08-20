package com.team581.trailblazer.constraints;

import edu.wpi.first.math.trajectory.TrapezoidProfile;

public record AutoConstraintOptions(
    /** Max linear velocity allowed in meters per second. Set to 0 to disable. */
    double maxLinearVelocity,
    /** Max angular velocity allowed in radians per second. Set to 0 to disable. */
    double maxAngularVelocity,
    /** Max linear acceleration allowed in meters per second squared. Set to 0 to disable. */
    double maxLinearAcceleration,
    /** Max angular acceleration allowed in radians per second squared. Set to 0 to disable. */
    double maxAngularAcceleration) {
  /** Default constraint options to use if no point or segment specific options are set. */
  public AutoConstraintOptions() {
    this(4.75, 71.5, 8.5, 35.2);
  }

  public AutoConstraintOptions withMaxLinearVelocity(double maxLinearVelocity) {
    return new AutoConstraintOptions(
        maxLinearVelocity, maxAngularVelocity(), maxLinearAcceleration(), maxAngularAcceleration());
  }

  public AutoConstraintOptions withMaxAngularVelocity(double maxAngularVelocity) {
    return new AutoConstraintOptions(
        maxLinearVelocity(), maxAngularVelocity, maxLinearAcceleration(), maxAngularAcceleration());
  }

  public AutoConstraintOptions withMaxLinearAcceleration(double maxLinearAcceleration) {
    return new AutoConstraintOptions(
        maxLinearVelocity(), maxAngularVelocity(), maxLinearAcceleration, maxAngularAcceleration());
  }

  public AutoConstraintOptions withMaxAngularAcceleration(double maxAngularAcceleration) {
    return new AutoConstraintOptions(
        maxLinearVelocity(), maxAngularVelocity(), maxLinearAcceleration(), maxAngularAcceleration);
  }

  public TrapezoidProfile.Constraints getLinearConstraints() {
    return new TrapezoidProfile.Constraints(maxLinearVelocity, maxLinearAcceleration);
  }

  public TrapezoidProfile.Constraints getAngularConstraints() {
    return new TrapezoidProfile.Constraints(maxAngularVelocity, maxAngularAcceleration);
  }
}
