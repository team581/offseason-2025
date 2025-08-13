package frc.robot.odometry;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.imu.ImuSubsystem;
import frc.robot.swerve.SwerveSubsystem;

public class CustomOdometry extends Odometry<SwerveModulePosition[]> {
  private static final Translation2d[] robotRelativeModulePositions = {
    new Translation2d(Inches.of(12), Inches.of(12)), // front-left
    new Translation2d(Inches.of(12), Inches.of(-12)), // front-right
    new Translation2d(Inches.of(-12), Inches.of(12)), // back-left
    new Translation2d(Inches.of(-12), Inches.of(-12)) // back-right
  };
  private static final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(robotRelativeModulePositions);
  private final ImuSubsystem imu;
  private final SwerveSubsystem swerve;

  public CustomOdometry(ImuSubsystem imu, SwerveSubsystem swerve) {
    super(
        kinematics,
        Rotation2d.fromDegrees(imu.getRobotHeading()),
        swerve.drivetrain.getState().ModulePositions,
        swerve.drivetrain.getState().Pose);

    this.imu = imu;
    this.swerve = swerve;
  }

  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }

  @Override public void resetPose(Pose2d newPose) {
    resetPosition(
        Rotation2d.fromDegrees(imu.getRobotHeading()),
        swerve.drivetrain.getState().ModulePositions,
        newPose);
  }

  public void updatePose() {
    update(
        Rotation2d.fromDegrees(imu.getRobotHeading()),
        swerve.drivetrain.getState().ModulePositions);
  }
}
