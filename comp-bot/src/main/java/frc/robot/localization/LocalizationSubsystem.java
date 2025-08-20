package frc.robot.localization;

import com.ctre.phoenix6.Utils;
import com.team581.math.MathHelpers;
import com.team581.trailblazer.LocalizationBase;
import com.team581.util.state_machines.StateMachine;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.estimator.PoseEstimator;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.config.FeatureFlags;
import frc.robot.fms.FmsSubsystem;
import frc.robot.imu.ImuSubsystem;
import frc.robot.odometry.CustomOdometry;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.vision.VisionSubsystem;
import frc.robot.vision.results.TagResult;

// TODO: Get the odometry to work with WPILIB(Swerve Drive Odometry class) instead of CTRE(Swerve
// Drivetrain class)
public class LocalizationSubsystem extends StateMachine<LocalizationState>
    implements LocalizationBase {
  private static final Vector<N3> MT1_VISION_STD_DEVS =
      VecBuilder.fill(
          RobotConfig.get().vision().xyStdDev(),
          RobotConfig.get().vision().xyStdDev(),
          RobotConfig.get().vision().thetaStdDev());
  private static final Vector<N3> MT2_VISION_STD_DEVS =
      VecBuilder.fill(
          RobotConfig.get().vision().xyStdDev(),
          RobotConfig.get().vision().xyStdDev(),
          Double.MAX_VALUE);
  private final ImuSubsystem imu;
  private final VisionSubsystem vision;
  private final SwerveSubsystem swerve;
  private final CustomOdometry customOdometry;
  private final PoseEstimator<?> poseEstimator;
  private Pose2d robotPose = Pose2d.kZero;
  // Currently using default std devs for odometry
  private static final Vector<N3> ODOMETRY_STATE_STD_DEVS = VecBuilder.fill(0.1, 0.1, 0.1);
  private static final Vector<N3> VISION_MEASURMENT_STD_DEVS = VecBuilder.fill(0, 0, 0);

  public LocalizationSubsystem(
      ImuSubsystem imu,
      VisionSubsystem vision,
      SwerveSubsystem swerve,
      CustomOdometry customOdometry) {
    super(SubsystemPriority.LOCALIZATION, LocalizationState.DEFAULT_STATE);
    this.swerve = swerve;
    this.imu = imu;
    this.vision = vision;
    this.customOdometry = customOdometry;

    this.poseEstimator =
        new PoseEstimator<>(
            customOdometry.getKinematics(),
            customOdometry,
            ODOMETRY_STATE_STD_DEVS,
            VISION_MEASURMENT_STD_DEVS);

    if (FeatureFlags.FIELD_CALIBRATION.getAsBoolean()) {
      SmartDashboard.putData(
          "FieldCalibration/ResetGyroTo180",
          Commands.runOnce(() -> resetGyro(Rotation2d.fromDegrees(180))).ignoringDisable(true));
      SmartDashboard.putData(
          "FieldCalibration/ResetGyroTo0",
          Commands.runOnce(() -> resetGyro(Rotation2d.fromDegrees(0))).ignoringDisable(true));
      SmartDashboard.putData(
          "FieldCalibration/ResetGyroTo90",
          Commands.runOnce(() -> resetGyro(Rotation2d.fromDegrees(90))).ignoringDisable(true));
      SmartDashboard.putData(
          "FieldCalibration/ResetGyroTo270",
          Commands.runOnce(() -> resetGyro(Rotation2d.fromDegrees(270))).ignoringDisable(true));
    }
  }

  @Override
  protected void collectInputs() {
    vision
        .getLeftFrontTagResult()
        .or(vision::getLeftBackTagResult)
        .ifPresent(this::ingestTagResult);
    vision.getRightTagResult().ifPresent(this::ingestTagResult);

    robotPose = poseEstimator.getEstimatedPosition();
  }

  @Override
  public Pose2d getPose() {
    return robotPose;
  }

  public Pose2d getPose(double timestamp) {
    var newTimestamp = Utils.fpgaToCurrentTime(timestamp);
    return poseEstimator.sampleAt(newTimestamp).orElseGet(this::getPose);
  }

  public Pose2d getLookaheadPose(double lookahead) {
    return MathHelpers.poseLookahead(getPose(), swerve.getFieldRelativeSpeeds(), lookahead);
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();

    DogLog.log("Localization/EstimatedPose", getPose());
  }

  private void ingestTagResult(TagResult result) {
    var visionPose = result.pose();

    if (!vision.seenTagRecentlyForReset() && FeatureFlags.MT_VISION_METHOD.getAsBoolean()) {
      resetPose(visionPose);
    }
    poseEstimator.addVisionMeasurement(visionPose, result.timestamp(), result.standardDevs());
  }

  private void resetGyro(Rotation2d gyroAngle) {
    imu.setAngle(gyroAngle.getDegrees());
    poseEstimator.resetRotation(gyroAngle);
  }

  public void resetPose(Pose2d estimatedPose) {
    // Reset the gyro when requested in teleop
    // Otherwise, if we are in auto, only reset it if we aren't already at the correct heading
    if (DriverStation.isTeleop()
        || !MathUtil.isNear(
            estimatedPose.getRotation().getDegrees(), imu.getRobotHeading(), 1.5, -180, 180)) {
      imu.setAngle(estimatedPose.getRotation().getDegrees());
    }

    poseEstimator.resetPose(estimatedPose);
  }

  public Command getZeroCommand() {
    return Commands.runOnce(
        () -> resetGyro(Rotation2d.fromDegrees((FmsSubsystem.isRedAlliance() ? 180 : 0))));
  }
}
