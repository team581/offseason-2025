package frc.robot.robot_manager.ground_manager;

import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.signals.S2StateValue;
import com.team581.util.state_machines.StateMachine;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.config.RobotConfig;
import frc.robot.intake.IntakeState;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.intake_deploy.DeployState;
import frc.robot.intake_deploy.DeploySubsystem;
import frc.robot.singulator.SingulatorState;
import frc.robot.singulator.SingulatorSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;

public class GroundManager extends StateMachine<GroundState> {
  public final IntakeSubsystem intake;
  public final DeploySubsystem deploy;
  public final SingulatorSubsystem singulator;

  private final CANdi topSensor;
  private final CANdi bottomSensor;

  private final Debouncer topDebouncer = RobotConfig.get().intake().topDebouncer();
  private final Debouncer bottomDebouncer = RobotConfig.get().intake().bottomDebouncer();

  private boolean topRaw = false;
  private boolean bottomRaw = false;
  private boolean topDebounced = false;
  private boolean bottomDebounced = false;

  public GroundManager(
      IntakeSubsystem intake,
      DeploySubsystem deploy,
      SingulatorSubsystem singulator,
      CANdi topSensor,
      CANdi bottomSensor) {
    super(
        SubsystemPriority.GROUND_MANAGER,
        RobotBase.isSimulation() ? GroundState.IDLE_NO_GP : GroundState.DEPLOY_NOT_HOMED);

    this.intake = intake;
    this.deploy = deploy;
    this.singulator = singulator;
    this.topSensor = topSensor;
    this.bottomSensor = bottomSensor;
  }

  @Override
  protected GroundState getNextState(GroundState currentState) {
    return switch (currentState) {
      case DEPLOY_HOMING ->
          deploy.getState() == DeployState.STOWED ? GroundState.IDLE_NO_GP : currentState;
      case INTAKING -> getHasGP() ? GroundState.IDLE_GP : currentState;
      default -> currentState;
    };
  }

  @Override
  protected void afterTransition(GroundState newState) {
    switch (newState) {
      case DEPLOY_HOMING -> {
        intake.setState(IntakeState.STOPPED);
        deploy.rehome();
        singulator.setState(SingulatorState.IDLE);
      }
      case DEPLOY_NOT_HOMED -> {
        intake.setState(IntakeState.STOPPED);
        deploy.setState(DeployState.UNHOMED);
        singulator.setState(SingulatorState.IDLE);
      }
      case IDLE_NO_GP, IDLE_GP -> {
        intake.setState(IntakeState.IDLE);
        deploy.setState(DeployState.STOWED);
        singulator.setState(SingulatorState.IDLE);
      }
      case INTAKING -> {
        intake.setState(IntakeState.INTAKING);
        deploy.setState(DeployState.FLOOR_INTAKE);
        singulator.setState(SingulatorState.INTAKING);
      }
      default -> {}
    }
  }

  private boolean homingOrUnhomed = true;

  @Override
  protected void collectInputs() {
    topRaw = topSensor.getS2State().getValue() == S2StateValue.High;
    bottomRaw = bottomSensor.getS2State().getValue() == S2StateValue.High;
    topDebounced = topDebouncer.calculate(topRaw);
    bottomDebounced = bottomDebouncer.calculate(bottomRaw);

    homingOrUnhomed =
        getState() == GroundState.DEPLOY_HOMING || getState() == GroundState.DEPLOY_NOT_HOMED;
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();

    DogLog.log("GroundManager/TopSensor/Debounced", topDebounced);
    DogLog.log("GroundManager/TopSensor/Raw", topRaw);
    DogLog.log("GroundManager/BottomSensor/Debounced", bottomDebounced);
    DogLog.log("GroundManager/BottomSensor/Raw", bottomRaw);
  }

  public boolean getHasGP() {
    return bottomDebounced || topDebounced;
  }

  public void rehomeRequest() {
    setStateFromRequest(GroundState.DEPLOY_HOMING);
  }

  public void intakeRequest() {
    if (homingOrUnhomed) {
      return;
    }

    setStateFromRequest(GroundState.INTAKING);
  }

  public void stowRequest() {
    if (homingOrUnhomed) {
      return;
    }

    if (getHasGP()) {
      setStateFromRequest(GroundState.IDLE_GP);
      return;
    }
    setStateFromRequest(GroundState.IDLE_NO_GP);
  }
}
