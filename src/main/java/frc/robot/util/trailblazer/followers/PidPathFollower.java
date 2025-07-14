package frc.robot.util.trailblazer.followers;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import frc.robot.util.MathHelpers;
import frc.robot.util.kinematics.PolarChassisSpeeds;
import frc.robot.util.trailblazer.constraints.AutoConstraintOptions;

public class PidPathFollower implements PathFollower {
  private static final double ALWAYS_FEASIBLE_LINEAR_VELOCITY = 0.05;

  private final PIDController velocityController;
  private final ProfiledPIDController rotationController;
  private double lastMaxLinearAcceleration =
      new AutoConstraintOptions().linearConstraints().maxAcceleration;
  private SlewRateLimiter linearAccelerationLimiter =
      new SlewRateLimiter(lastMaxLinearAcceleration);

  public PidPathFollower(PIDController velocityController, PIDController rotationController) {
    this.velocityController = velocityController;
    this.rotationController =
        new ProfiledPIDController(
            rotationController.getP(),
            rotationController.getI(),
            rotationController.getD(),
            new AutoConstraintOptions().angularConstraints());

    this.rotationController.enableContinuousInput(-Math.PI, Math.PI);
  }

  @Override
  public PolarChassisSpeeds calculateSpeeds(
      Pose2d currentPose,
      Pose2d targetPose,
      PolarChassisSpeeds currentSpeeds,
      AutoConstraintOptions constraints) {
    var rawLinearVelocity =
        Math.abs(
            velocityController.calculate(
                currentPose.getTranslation().getDistance(targetPose.getTranslation()), 0));

    var clampedLinearVelocity =
        Math.min(rawLinearVelocity, constraints.linearConstraints().maxVelocity);

    if (lastMaxLinearAcceleration != constraints.linearConstraints().maxAcceleration) {
      lastMaxLinearAcceleration = constraints.linearConstraints().maxAcceleration;
      linearAccelerationLimiter =
          new SlewRateLimiter(
              lastMaxLinearAcceleration,
              -lastMaxLinearAcceleration,
              currentSpeeds.vMetersPerSecond);
    }

    var feasibleLinearVelocity = linearAccelerationLimiter.calculate(clampedLinearVelocity);

    // Ensure that you can always accelerate from 0 to some reasonable velocity
    if (feasibleLinearVelocity < clampedLinearVelocity) {
      feasibleLinearVelocity = Math.max(feasibleLinearVelocity, ALWAYS_FEASIBLE_LINEAR_VELOCITY);
    }

    return new PolarChassisSpeeds(
        feasibleLinearVelocity,
        MathHelpers.getDriveDirection(targetPose, currentPose),
        rotationController.calculate(
            currentPose.getRotation().getRadians(),
            new TrapezoidProfile.State(targetPose.getRotation().getRadians(), 0),
            constraints.angularConstraints()));
  }
}
