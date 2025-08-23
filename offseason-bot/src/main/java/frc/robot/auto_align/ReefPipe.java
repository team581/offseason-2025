package frc.robot.auto_align;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.fms.FmsSubsystem;

public enum ReefPipe {
  PIPE_A(new Pose2d(3.71, 4.19, Rotation2d.kZero), new Pose2d(13.84, 3.86, Rotation2d.k180deg)),
  PIPE_B(new Pose2d(3.71, 3.86, Rotation2d.kZero), new Pose2d(13.84, 4.19, Rotation2d.k180deg)),
  PIPE_C(
      new Pose2d(3.96, 3.43, Rotation2d.fromDegrees(60)),
      new Pose2d(13.59, 4.62, Rotation2d.fromDegrees(240))),
  PIPE_D(
      new Pose2d(4.24, 3.27, Rotation2d.fromDegrees(60)),
      new Pose2d(13.31, 4.78, Rotation2d.fromDegrees(240))),
  PIPE_E(
      new Pose2d(4.74, 3.27, Rotation2d.fromDegrees(120)),
      new Pose2d(12.81, 4.78, Rotation2d.fromDegrees(300))),
  PIPE_F(
      new Pose2d(5.02, 3.43, Rotation2d.fromDegrees(120)),
      new Pose2d(12.53, 4.62, Rotation2d.fromDegrees(300))),
  PIPE_G(new Pose2d(5.27, 3.86, Rotation2d.k180deg), new Pose2d(12.29, 4.19, Rotation2d.kZero)),
  PIPE_H(new Pose2d(5.27, 4.19, Rotation2d.k180deg), new Pose2d(12.29, 3.86, Rotation2d.kZero)),
  PIPE_I(
      new Pose2d(5.02, 4.62, Rotation2d.fromDegrees(240)),
      new Pose2d(12.53, 3.43, Rotation2d.fromDegrees(60))),
  PIPE_J(
      new Pose2d(4.74, 4.78, Rotation2d.fromDegrees(240)),
      new Pose2d(12.81, 3.27, Rotation2d.fromDegrees(60))),
  PIPE_K(
      new Pose2d(4.24, 4.78, Rotation2d.fromDegrees(300)),
      new Pose2d(13.31, 3.27, Rotation2d.fromDegrees(120))),
  PIPE_L(
      new Pose2d(3.96, 4.62, Rotation2d.fromDegrees(300)),
      new Pose2d(13.59, 3.43, Rotation2d.fromDegrees(120)));

  private final Pose2d redPose;
  private final Pose2d bluePose;

  ReefPipe(Pose2d bluePose, Pose2d redPose) {
    this.redPose = redPose;
    this.bluePose = bluePose;
  }

  public Pose2d getPose(ReefPipeLevel level) {
    return FmsSubsystem.isRedAlliance()
        ? redPose
        : bluePose;
  }

  public Pose2d getPose(ReefPipeLevel level, boolean isRedAlliance) {
    return isRedAlliance ? redPose : bluePose;
  }

  public Pose2d getPose(ReefPipeLevel level, Pose2d robotPose) {
    return getPose(level, robotPose.getX() > (17.5 / 2));
  }
}
