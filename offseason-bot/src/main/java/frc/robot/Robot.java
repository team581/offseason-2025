package frc.robot;

import com.ctre.phoenix6.SignalLogger;
import com.team581.GlobalConfig;
import com.team581.trailblazer.Trailblazer;
import com.team581.util.scheduling.LifecycleSubsystemManager;
import com.team581.util.tuning.ElasticLayoutUtil;
import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.config.FeatureFlags;
import frc.robot.config.RobotConfig;
import frc.robot.fms.FmsSubsystem;
import frc.robot.generated.BuildConstants;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.intake_deploy.DeploySubsystem;
import frc.robot.robot_manager.RobotCommands;
import frc.robot.robot_manager.ground_manager.GroundManager;
import frc.robot.singulator.SingulatorSubsystem;

public class Robot extends TimedRobot {
  private Command autonomousCommand = Commands.none();
  private final FmsSubsystem fms = new FmsSubsystem();
  private final Hardware hardware = new Hardware();

  private final IntakeSubsystem intake = new IntakeSubsystem(hardware.intakeMotor);
  private final DeploySubsystem deploy = new DeploySubsystem(hardware.deployMotor);
  private final SingulatorSubsystem singulator = new SingulatorSubsystem(hardware.leftSingulatorMotor, hardware.rightSingulatorMotor);

  private final GroundManager groundManager = new GroundManager(intake, deploy, singulator, hardware.intakeTopCANdi, hardware.intakeBottomCANdi);

  private final RobotCommands actions = new RobotCommands(groundManager);

  public Robot() {
    System.out.println("roboRIO serial number: " + RobotConfig.SERIAL_NUMBER);

    DriverStation.silenceJoystickConnectionWarning(RobotBase.isSimulation());

    SignalLogger.start();
    SignalLogger.setPath("/media/sda1/hoot/");

    DogLog.setOptions(
        new DogLogOptions()
            .withCaptureDs(true)
            .withNtPublish(GlobalConfig.IS_DEVELOPMENT)
            .withNtTunables(GlobalConfig.IS_DEVELOPMENT));
    // DogLog.setPdh(hardware.pdh);

    // Record metadata
    DogLog.log("Metadata/ProjectName", BuildConstants.MAVEN_NAME);
    DogLog.log("Metadata/RoborioSerialNumber", RobotConfig.SERIAL_NUMBER);
    DogLog.log("Metadata/RobotName", RobotConfig.get().robotName());
    DogLog.log("Metadata/BuildDate", BuildConstants.BUILD_DATE);
    DogLog.log("Metadata/GitSHA", BuildConstants.GIT_SHA);
    DogLog.log("Metadata/GitDate", BuildConstants.GIT_DATE);
    DogLog.log("Metadata/GitBranch", BuildConstants.GIT_BRANCH);

    switch (BuildConstants.DIRTY) {
      case 0 -> DogLog.log("Metadata/GitDirty", "All changes committed");
      case 1 -> DogLog.log("Metadata/GitDirty", "Uncomitted changes");
      default -> DogLog.log("Metadata/GitDirty", "Unknown");
    }

    // This must be run before any commands are scheduled
    LifecycleSubsystemManager.ready();

    configureBindings();

    ElasticLayoutUtil.onBoot();
  }

  @Override
  public void robotInit() {}

  @Override
  public void robotPeriodic() {
    DogLog.timeEnd("Scheduler/TimeSinceLastLoop");
    DogLog.time("Scheduler/TimeSinceLastLoop");

    DogLog.time("Scheduler/CommandSchedulerPeriodic");
    CommandScheduler.getInstance().run();
    DogLog.timeEnd("Scheduler/CommandSchedulerPeriodic");
    LifecycleSubsystemManager.log();

    if (RobotController.getBatteryVoltage() < 12.5) {
      DogLog.logFault("Battery voltage low", AlertType.kWarning);
    } else {
      DogLog.clearFault("Battery voltage low");
    }

    // if (FeatureFlags.FIELD_CALIBRATION.getAsBoolean()) {
    //   fieldCalibrationUtil.log();
    // }
  }

  @Override
  public void disabledInit() {
    ElasticLayoutUtil.onDisable();
  }

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    // autonomousCommand = autos.getAutoCommand();
    // autonomousCommand.schedule();

    ElasticLayoutUtil.onEnable();
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    autonomousCommand.cancel();

    ElasticLayoutUtil.onEnable();
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();
  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  private void configureBindings() {
    // swerve.setDefaultCommand(
    //     swerve
    //         .run(
    //             () -> {
    //               if (DriverStation.isTeleop()) {
    //                 swerve.driveTeleop(
    //                     hardware.driverController.getLeftX(),
    //                     hardware.driverController.getLeftY(),
    //                     hardware.driverController.getRightX());
    //               }
    //             })
    //         .ignoringDisable(true)
    //         .withName("DefaultSwerveCommand"));

    hardware
        .driverController
        .leftTrigger()
        .onTrue(actions.groundIntakeCommand());
    hardware
        .driverController
        .leftTrigger()
        .onTrue(actions.stowCommand());
  }
}
