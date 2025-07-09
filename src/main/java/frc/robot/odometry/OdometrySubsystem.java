package frc.robot.odometry;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;

public class OdometrySubsystem {
  private static final Translation2d[] modulePositions = {
    new Translation2d(12, 12), // front-left
    new Translation2d(12, -12), // front-right
    new Translation2d(-12, 12), // back-left
    new Translation2d(-12, -12) // back-right
  };
  private final SwerveDriveKinematics kinematics = new SwerveDriveKinematics(modulePositions);
  private final Rotation2d gyroAngle = new Rotation2d();

  // private SwerveDriveOdometry wpiOdometry = new SwerveDriveOdometry(kinematics, gyroAngle, new
  // SwerveModulePosition[]);

  public OdometrySubsystem() {}

  public void resetPosition(Rotation2d gyroAngle, SwerveModulePosition[] modulePositions) {
    // wpiOdometry.resetPosition(gyroAngle, modulePositions);
  }
}
