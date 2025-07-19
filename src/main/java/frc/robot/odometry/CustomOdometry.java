package frc.robot.odometry;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.Odometry;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.imu.ImuSubsystem;
import frc.robot.swerve.SwerveSubsystem;

// TODO: Make custom odometry extend wpi odometry class, then implement in localization
// TODO: Check math and methods used for gyro angle calculations

public class CustomOdometry extends Odometry {
  private static final Translation2d[] robotRelativeModulePositions = {
    new Translation2d(Inches.of(12), Inches.of(12)), // front-left
    new Translation2d(Inches.of(12), Inches.of(-12)), // front-right
    new Translation2d(Inches.of(-12), Inches.of(12)), // back-left
    new Translation2d(Inches.of(-12), Inches.of(-12)) // back-right
  };
  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(robotRelativeModulePositions);
  private Rotation2d gyroAngle;
  private SwerveModulePosition[] fieldRelativeModulePositions;
  private final ImuSubsystem imu;
  private final SwerveSubsystem swerve;
  private final SwerveDriveOdometry wpiOdometry;

  public CustomOdometry(ImuSubsystem imu, SwerveSubsystem swerve) {
    this();
    this.imu = imu;
    this.swerve = swerve;

    this.gyroAngle = Rotation2d.fromDegrees(imu.getRobotHeading());
    this.fieldRelativeModulePositions = swerve.drivetrain.getState().ModulePositions;
    this.wpiOdometry = new SwerveDriveOdometry(kinematics, gyroAngle, fieldRelativeModulePositions);
  }

  public SwerveDriveKinematics getKinematics() {
    return kinematics;
  }

  public void resetPose(Rotation2d gyroAngle, SwerveModulePosition[] modulePositions, Pose2d pose) {
    wpiOdometry.resetPosition(gyroAngle, modulePositions, pose);
  }

  public void updatePose() {
    wpiOdometry.update(
        Rotation2d.fromDegrees(imu.getRobotHeading()),
        swerve.drivetrain.getState().ModulePositions);
  }
}
