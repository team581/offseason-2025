package frc.robot.swerve;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import com.ctre.phoenix6.swerve.SwerveModule.DriveRequestType;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import com.team581.GlobalConfig;
import com.team581.math.ControllerHelpers;
import com.team581.trailblazer.SwerveBase;
import dev.doglog.DogLog;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.config.RobotConfig;
import frc.robot.fms.FmsSubsystem;
import frc.robot.generated.CompBotTunerConstants;
import frc.robot.generated.PracticeBotTunerConstants;
import frc.robot.generated.PracticeBotTunerConstants.TunerSwerveDrivetrain;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;
import java.util.Map;

public class SwerveSubsystem extends StateMachine<SwerveState> implements SwerveBase {
  private static final double LEFT_JOYSTICK_EXPONENT = 2;
  private static final double RIGHT_JOYSTICK_EXPONENT = 2;

  public static final double MaxSpeed = 4.75;
  private static final double maxAngularRate = Units.rotationsToRadians(4);
  private static final Rotation2d TELEOP_MAX_ANGULAR_RATE = Rotation2d.fromRotations(2);

  private static final double SIM_LOOP_PERIOD = 0.005; // 5 ms

  private static final PhoenixPIDController ORIGINAL_HEADING_PID =
      RobotConfig.get().swerve().snapController();

  private static final InterpolatingDoubleTreeMap ELEVATOR_HEIGHT_TO_SLOW_MODE =
      InterpolatingDoubleTreeMap.ofEntries(Map.entry(0.0, 1.0));

  public final TunerSwerveDrivetrain drivetrain =
      RobotConfig.IS_PRACTICE_BOT
          ? new TunerSwerveDrivetrain(
              PracticeBotTunerConstants.DrivetrainConstants,
              PracticeBotTunerConstants.FrontLeft,
              PracticeBotTunerConstants.FrontRight,
              PracticeBotTunerConstants.BackLeft,
              PracticeBotTunerConstants.BackRight)
          : new TunerSwerveDrivetrain(
              CompBotTunerConstants.DrivetrainConstants,
              CompBotTunerConstants.FrontLeft,
              CompBotTunerConstants.FrontRight,
              CompBotTunerConstants.BackLeft,
              CompBotTunerConstants.BackRight);

  public final Pigeon2 drivetrainPigeon = drivetrain.getPigeon2();

  private final SwerveRequest.FieldCentric drive =
      new SwerveRequest.FieldCentric()
          // I want field-centric driving in open loop
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
          .withDeadband(MaxSpeed * 0.015)
          .withRotationalDeadband(maxAngularRate * 0.015);

  private final SwerveRequest.FieldCentricFacingAngle driveToAngle =
      new SwerveRequest.FieldCentricFacingAngle()
          .withDriveRequestType(DriveRequestType.OpenLoopVoltage)
          .withDeadband(MaxSpeed * 0.01)
          .withRotationalDeadband(maxAngularRate * 0.005)
          .withHeadingPID(
              ORIGINAL_HEADING_PID.getP(), ORIGINAL_HEADING_PID.getI(), ORIGINAL_HEADING_PID.getD())
          .withMaxAbsRotationalRate(maxAngularRate);

  private double lastSimTime;
  private Notifier simNotifier = null;

  private SwerveDriveState drivetrainState = new SwerveDriveState();
  private ChassisSpeeds robotRelativeSpeeds = new ChassisSpeeds();
  private ChassisSpeeds fieldRelativeSpeeds = new ChassisSpeeds();
  private double goalSnapAngle = 0;

  /** The latest requested teleop speeds. */
  private ChassisSpeeds teleopSpeeds = new ChassisSpeeds();

  private ChassisSpeeds autoSpeeds = new ChassisSpeeds();

  private ChassisSpeeds coralAssistSpeeds = new ChassisSpeeds();

  private ChassisSpeeds autoAlignSpeeds = new ChassisSpeeds();

  private final ChassisSpeeds previousSpeeds = new ChassisSpeeds();
  private static final double PREVIOUS_TIMESTAMP = 0.0;
  private final Timer timeSinceAutoSpeeds = new Timer();
  private double teleopSlowModePercent = 0.0;
  private double rawControllerXValue = 0.0;
  private double rawControllerYValue = 0.0;

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return robotRelativeSpeeds;
  }

  @Override
  public ChassisSpeeds getFieldRelativeSpeeds() {
    return fieldRelativeSpeeds;
  }

  public void setSnapToAngle(double angle) {
    goalSnapAngle = angle;

    // We don't necessarily set auto swerve speeds every loop, so this ensures we are always snapped
    // to the right angle during auto. Teleop doesn't need this since teleop speeds are constantly
    // fed into swerve.
    if (DriverStation.isAutonomous()) {
      sendSwerveRequest();
    }
  }

  private double elevatorHeight;

  public SwerveSubsystem() {
    super(SubsystemPriority.SWERVE, SwerveState.TELEOP);

    if (Utils.isSimulation()) {
      startSimThread();
    }

    driveToAngle.HeadingController.setTolerance(0.01);

    drivetrain.setStateStdDevs(new Matrix<>(VecBuilder.fill(0.003, 0.003, 0.002)));
    timeSinceAutoSpeeds.start();
  }

  @Override
  public void setFieldRelativeAutoSpeeds(ChassisSpeeds speeds) {
    autoSpeeds = speeds;
    timeSinceAutoSpeeds.reset();
    sendSwerveRequest();
  }

  public void setRobotRelativeAutoSpeeds(ChassisSpeeds speeds) {
    setFieldRelativeAutoSpeeds(
        ChassisSpeeds.fromRobotRelativeSpeeds(speeds, drivetrainState.Pose.getRotation()));
  }

  public void setFieldRelativeCoralAssistSpeeds(ChassisSpeeds speeds) {
    coralAssistSpeeds = speeds;
  }

  public void setAutoAlignSpeeds(ChassisSpeeds speeds) {
    autoAlignSpeeds = speeds;
    sendSwerveRequest();
  }

  @Override
  protected SwerveState getNextState(SwerveState currentState) {
    // Ensure that we are in an auto state during auto, and a teleop state during teleop
    return switch (currentState) {
      case AUTO, TELEOP -> DriverStation.isAutonomous() ? SwerveState.AUTO : SwerveState.TELEOP;
      case REEF_ALIGN_TELEOP ->
          DriverStation.isAutonomous() ? SwerveState.AUTO : SwerveState.REEF_ALIGN_TELEOP;
      case CORAL_ALIGN -> DriverStation.isAutonomous() ? SwerveState.AUTO : SwerveState.CORAL_ALIGN;
      case AUTO_SNAPS, TELEOP_SNAPS ->
          DriverStation.isAutonomous() ? SwerveState.AUTO_SNAPS : SwerveState.TELEOP_SNAPS;
      case CLIMBING -> DriverStation.isAutonomous() ? SwerveState.AUTO : SwerveState.CLIMBING;
    };
  }

  public void driveTeleop(double x, double y, double theta) {
    rawControllerXValue = x;
    rawControllerYValue = y;

    if (GlobalConfig.IS_DEVELOPMENT) {
      DogLog.log("Swerve/RawJoystickInput", new Translation2d(x, y));
    }

    var leftJoystickMagnitude =
        ControllerHelpers.getJoystickMagnitude(x, y, LEFT_JOYSTICK_EXPONENT);
    var rightJoystickMagnitude =
        ControllerHelpers.getJoystickMagnitude(theta, 0, RIGHT_JOYSTICK_EXPONENT);

    var translation = new Translation2d(leftJoystickMagnitude, new Rotation2d(x, y));
    var rotation = new Translation2d(rightJoystickMagnitude, new Rotation2d(theta, 0));

    var leftX = translation.getX();
    var leftY = translation.getY();
    var rightX = rotation.getX();

    if (RobotConfig.get().swerve().invertRotation()) {
      rightX *= -1.0;
    }

    if (RobotConfig.get().swerve().invertX()) {
      leftX *= -1.0;
    }

    if (RobotConfig.get().swerve().invertY()) {
      leftY *= -1.0;
    }

    if (FmsSubsystem.isRedAlliance()) {
      leftX *= -1.0;
      leftY *= -1.0;
    }

    teleopSpeeds =
        new ChassisSpeeds(
            -1.0 * leftY * MaxSpeed * teleopSlowModePercent,
            leftX * MaxSpeed * teleopSlowModePercent,
            rightX * TELEOP_MAX_ANGULAR_RATE.getRadians() * teleopSlowModePercent);

    sendSwerveRequest();
  }

  @Override
  protected void collectInputs() {
    drivetrainState = drivetrain.getState();
    robotRelativeSpeeds = drivetrainState.Speeds;
    fieldRelativeSpeeds = calculateFieldRelativeSpeeds();
    teleopSlowModePercent = ELEVATOR_HEIGHT_TO_SLOW_MODE.get(elevatorHeight);
  }

  public ChassisSpeeds getTeleopSpeeds() {
    return teleopSpeeds;
  }

  private ChassisSpeeds calculateFieldRelativeSpeeds() {
    return ChassisSpeeds.fromRobotRelativeSpeeds(
        robotRelativeSpeeds, drivetrainState.Pose.getRotation());
  }

  private void sendSwerveRequest() {
    switch (getState()) {
      case TELEOP ->
          drivetrain.setControl(
              drive
                  .withVelocityX(teleopSpeeds.vxMetersPerSecond)
                  .withVelocityY(teleopSpeeds.vyMetersPerSecond)
                  .withRotationalRate(teleopSpeeds.omegaRadiansPerSecond)
                  .withDriveRequestType(DriveRequestType.OpenLoopVoltage));
      case TELEOP_SNAPS -> {
        if (teleopSpeeds.omegaRadiansPerSecond == 0) {
          drivetrain.setControl(
              driveToAngle
                  .withVelocityX(teleopSpeeds.vxMetersPerSecond)
                  .withVelocityY(teleopSpeeds.vyMetersPerSecond)
                  .withTargetDirection(Rotation2d.fromDegrees(goalSnapAngle))
                  .withMaxAbsRotationalRate(
                      TELEOP_MAX_ANGULAR_RATE.getRadians() * teleopSlowModePercent)
                  .withDriveRequestType(DriveRequestType.OpenLoopVoltage));

        } else {
          drivetrain.setControl(
              drive
                  .withVelocityX(teleopSpeeds.vxMetersPerSecond)
                  .withVelocityY(teleopSpeeds.vyMetersPerSecond)
                  .withRotationalRate(teleopSpeeds.omegaRadiansPerSecond)
                  .withDriveRequestType(DriveRequestType.OpenLoopVoltage));
        }
      }
      case CORAL_ALIGN ->
          drivetrain.setControl(
              drive
                  .withVelocityX(coralAssistSpeeds.vxMetersPerSecond)
                  .withVelocityY(coralAssistSpeeds.vyMetersPerSecond)
                  .withRotationalRate(coralAssistSpeeds.omegaRadiansPerSecond)
                  .withDriveRequestType(DriveRequestType.OpenLoopVoltage));
      case REEF_ALIGN_TELEOP -> {
        drivetrain.setControl(
            drive
                .withVelocityX(autoAlignSpeeds.vxMetersPerSecond)
                .withVelocityY(autoAlignSpeeds.vyMetersPerSecond)
                .withRotationalRate(autoAlignSpeeds.omegaRadiansPerSecond)
                .withDriveRequestType(DriveRequestType.Velocity));
      }

      case AUTO ->
          drivetrain.setControl(
              drive
                  .withVelocityX(autoSpeeds.vxMetersPerSecond)
                  .withVelocityY(autoSpeeds.vyMetersPerSecond)
                  .withRotationalRate(autoSpeeds.omegaRadiansPerSecond)
                  .withDriveRequestType(DriveRequestType.Velocity));
      case AUTO_SNAPS -> {
        drivetrain.setControl(
            driveToAngle
                .withVelocityX(autoSpeeds.vxMetersPerSecond)
                .withVelocityY(autoSpeeds.vyMetersPerSecond)
                .withTargetDirection(Rotation2d.fromDegrees(goalSnapAngle))
                .withMaxAbsRotationalRate(maxAngularRate)
                .withDriveRequestType(DriveRequestType.Velocity));
      }
      case CLIMBING -> {
        if (teleopSpeeds.omegaRadiansPerSecond == 0) {
          drivetrain.setControl(
              driveToAngle
                  .withVelocityX(teleopSpeeds.vxMetersPerSecond / 2)
                  .withVelocityY(teleopSpeeds.vyMetersPerSecond / 2)
                  .withMaxAbsRotationalRate(
                      TELEOP_MAX_ANGULAR_RATE.getRadians() * teleopSlowModePercent)
                  .withTargetDirection(Rotation2d.fromDegrees(goalSnapAngle))
                  .withDriveRequestType(DriveRequestType.OpenLoopVoltage));
        } else {
          drivetrain.setControl(
              drive
                  .withVelocityX(teleopSpeeds.vxMetersPerSecond / 2)
                  .withVelocityY(teleopSpeeds.vyMetersPerSecond / 2)
                  .withRotationalRate(teleopSpeeds.omegaRadiansPerSecond)
                  .withDriveRequestType(DriveRequestType.OpenLoopVoltage));
        }
      }
    }
  }

  public void normalDriveRequest() {
    if (DriverStation.isAutonomous()) {
      setStateFromRequest(SwerveState.AUTO);
    } else {
      setStateFromRequest(SwerveState.TELEOP);
    }
  }

  public void coralAlignDriveRequest() {
    if (DriverStation.isAutonomous()) {
      setStateFromRequest(SwerveState.AUTO);
    } else {
      setStateFromRequest(SwerveState.CORAL_ALIGN);
    }
  }

  public Translation2d getControllerValues() {
    if (getState() != SwerveState.REEF_ALIGN_TELEOP) {
      return Translation2d.kZero;
    }

    return new Translation2d(
        ControllerHelpers.getJoystickMagnitude(
            rawControllerXValue, rawControllerYValue, LEFT_JOYSTICK_EXPONENT),
        new Rotation2d(rawControllerXValue, rawControllerYValue));
  }

  public void snapsDriveRequest(double snapAngle) {
    setSnapToAngle(snapAngle);

    if (DriverStation.isAutonomous()) {
      setStateFromRequest(SwerveState.AUTO_SNAPS);
    } else {
      setStateFromRequest(SwerveState.TELEOP_SNAPS);
    }
  }

  public void scoringAlignmentRequest(double snapAngle) {
    if (DriverStation.isAutonomous()) {
      normalDriveRequest();
    } else {
      setSnapToAngle(snapAngle);
      setStateFromRequest(SwerveState.REEF_ALIGN_TELEOP);
    }
  }

  public void climbRequest(double snapAngle) {
    setSnapToAngle(snapAngle);
    setStateFromRequest(SwerveState.CLIMBING);
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();

    DogLog.log("Swerve/SnapAngle", goalSnapAngle);
    DogLog.log("Swerve/ModuleStates", drivetrainState.ModuleStates);
    DogLog.log("Swerve/ModuleTargets", drivetrainState.ModuleTargets);
    DogLog.log("Swerve/RobotRelativeSpeeds", drivetrainState.Speeds);

    if (timeSinceAutoSpeeds.hasElapsed(0.25) && DriverStation.isAutonomous()) {
      DogLog.logFault("Timeout since auto speeds last set");
    } else {
      DogLog.clearFault("Timeout since auto speeds last set");
    }
  }

  private void startSimThread() {
    lastSimTime = Utils.getCurrentTimeSeconds();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    simNotifier =
        new Notifier(
            () -> {
              double currentTime = Utils.getCurrentTimeSeconds();
              double deltaTime = currentTime - lastSimTime;
              lastSimTime = currentTime;

              /* use the measured time delta, get battery voltage from WPILib */
              drivetrain.updateSimState(deltaTime, RobotController.getBatteryVoltage());
            });
    simNotifier.startPeriodic(SIM_LOOP_PERIOD);
  }

  public void setElevatorHeight(double height) {
    elevatorHeight = height;
  }
}
