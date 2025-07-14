package frc.robot.util.trailblazer.constraints;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;

public record AutoConstraintOptions(
    TrapezoidProfile.Constraints linearConstraints,
    TrapezoidProfile.Constraints angularConstraints) {
  /** Default constraint options to use if no point or segment specific options are set. */
  public AutoConstraintOptions() {
    this(
        new TrapezoidProfile.Constraints(5, 4),
        new TrapezoidProfile.Constraints(Units.rotationsToRadians(4), Units.rotationsToRadians(2)));
  }

  public AutoConstraintOptions withMaxLinearVelocity(double maxLinearVelocity) {
    return new AutoConstraintOptions(
        new TrapezoidProfile.Constraints(maxLinearVelocity, linearConstraints.maxAcceleration),
        angularConstraints);
  }

  public AutoConstraintOptions withMaxAngularVelocity(double maxAngularVelocity) {
    return new AutoConstraintOptions(
        linearConstraints,
        new TrapezoidProfile.Constraints(maxAngularVelocity, angularConstraints.maxAcceleration));
  }

  public AutoConstraintOptions withMaxLinearAcceleration(double maxLinearAcceleration) {
    return new AutoConstraintOptions(
        new TrapezoidProfile.Constraints(linearConstraints.maxVelocity, maxLinearAcceleration),
        angularConstraints);
  }

  public AutoConstraintOptions withMaxAngularAcceleration(double maxAngularAcceleration) {
    return new AutoConstraintOptions(
        linearConstraints,
        new TrapezoidProfile.Constraints(angularConstraints.maxVelocity, maxAngularAcceleration));
  }
}
