package frc.robot.odometry;

import static edu.wpi.first.units.Units.Inches;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveDriveOdometry;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import frc.robot.imu.ImuSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;

public class OdometrySubsystem extends StateMachine<OdometryState> {
  private static final Translation2d[] robotRelativeModulePositions = {
    new Translation2d(Inches.of(12), Inches.of(12)), // front-left
    new Translation2d(Inches.of(12), Inches.of(-12)), // front-right
    new Translation2d(Inches.of(-12), Inches.of(12)), // back-left
    new Translation2d(Inches.of(-12), Inches.of(-12)) // back-right
  };
  private final SwerveDriveKinematics kinematics =
      new SwerveDriveKinematics(robotRelativeModulePositions);
  private final ImuSubsystem imu;
  private final SwerveSubsystem swerve;
  private final SwerveDriveOdometry wpiOdometry;

  public OdometrySubsystem(ImuSubsystem imu, SwerveSubsystem swerve) {
    super(SubsystemPriority.ODOMETRY, OdometryState.DEFAULT_STATE);
    this.imu = imu;
    this.swerve = swerve;

     Rotation2d initialHeading = Rotation2d.fromDegrees(imu.getRobotHeading());
     SwerveModulePosition[] fieldRelativeModulePositions =
        swerve.drivetrain.getState().ModulePositions;
    this.wpiOdometry =
        new SwerveDriveOdometry(kinematics, initialHeading, fieldRelativeModulePositions);
  }

  public void resetPosition() {}

  public void update() {}
}
