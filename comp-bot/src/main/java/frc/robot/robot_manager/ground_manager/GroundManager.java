package frc.robot.robot_manager.ground_manager;

import com.team581.util.state_machines.StateMachine;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.intake.IntakeState;
import frc.robot.intake.IntakeSubsystem;
import frc.robot.intake_deploy.DeployState;
import frc.robot.intake_deploy.DeploySubsystem;
import frc.robot.util.scheduling.SubsystemPriority;

public class GroundManager extends StateMachine<GroundState> {
  public final DeploySubsystem deploy;
  public final IntakeSubsystem intake;
  private boolean hasCoral = false;

  public GroundManager(DeploySubsystem deploy, IntakeSubsystem intake) {
    super(SubsystemPriority.GROUND_MANAGER, GroundState.IDLE_EMPTY);
    this.deploy = deploy;
    this.intake = intake;
  }

  @Override
  protected GroundState getNextState(GroundState currentState) {
    return switch (currentState) {
      case INTAKING -> hasCoral ? GroundState.IDLE_CORAL : currentState;
      case HANDOFF_RELEASE, L1_SCORE, L1_HARD_SCORE ->
          !hasCoral ? GroundState.IDLE_EMPTY : currentState;
      case REHOME_DEPLOY -> deploy.atGoal() ? GroundState.IDLE_EMPTY : currentState;
      case INTAKE_THEN_HANDOFF_WAIT -> hasCoral ? GroundState.HANDOFF_WAIT : currentState;
      default -> currentState;
    };
  }

  @Override
  protected void afterTransition(GroundState newState) {
    switch (newState) {
      case IDLE_EMPTY -> {
        deploy.setState(DeployState.STOWED);
        intake.setState(IntakeState.IDLE_NO_GP);
      }
      case IDLE_CORAL -> {
        deploy.setState(DeployState.STOWED);
        intake.setState(IntakeState.IDLE_GP);
      }
      case INTAKING, INTAKE_THEN_HANDOFF_WAIT -> {
        deploy.setState(DeployState.FLOOR_INTAKE);
        intake.setState(IntakeState.INTAKING);
      }
      case L1_WAIT, L1_HARD_WAIT -> {
        deploy.setState(DeployState.L1_SCORE);
        intake.setState(IntakeState.IDLE_GP);
      }
      case L1_SCORE -> {
        deploy.setState(DeployState.L1_SCORE);
        intake.setState(IntakeState.SCORING);
      }
      case L1_HARD_SCORE -> {
        deploy.setState(DeployState.L1_SCORE);
        intake.setState(IntakeState.HARD_SCORING);
      }
      case HANDOFF_WAIT -> {
        deploy.setState(DeployState.HANDOFF);
        intake.setState(IntakeState.IDLE_GP);
      }
      case HANDOFF_RELEASE -> {
        deploy.setState(DeployState.HANDOFF);
        intake.setState(IntakeState.CORAL_HANDOFF);
      }
      case UNJAM -> {
        deploy.setState(DeployState.UNJAM);
        intake.setState(IntakeState.UNJAM);
      }
      case REHOME_DEPLOY -> {
        intake.setState(IntakeState.IDLE_NO_GP);
        deploy.setState(DeployState.HOMING);
      }
      case CLIMB -> {
        intake.setState(IntakeState.IDLE_NO_GP);
        deploy.setState(DeployState.FLOOR_INTAKE);
      }
    }
  }

  @Override
  protected void collectInputs() {
    hasCoral = intake.getHasGP();
  }

  private void setState(GroundState newState) {
    switch (deploy.getState()) {
      case UNHOMED, HOMING -> {}
      default -> setStateFromRequest(newState);
    }
  }

  public boolean hasCoral() {
    return hasCoral;
  }

  public void intakeRequest() {
    switch (getState()) {
      // case L1_WAIT, L1_HARD_WAIT -> setState(GroundState.L1_HARD_SCORE);

      case HANDOFF_WAIT, HANDOFF_RELEASE -> intakeThenHandoffRequest();
      default -> setState(GroundState.INTAKING);
    }
  }

  public void hardL1WaitRequest() {
    switch (getState()) {
      case L1_WAIT, L1_HARD_WAIT -> setState(GroundState.L1_HARD_WAIT);
      default -> setState(GroundState.L1_WAIT);
    }
  }

  public void l1WaitRequest() {
    setState(GroundState.L1_WAIT);
  }

  public void l1Request() {
    switch (getState()) {
      case L1_WAIT, L1_HARD_WAIT -> setState(GroundState.L1_SCORE);
      default -> setState(GroundState.L1_WAIT);
    }
  }

  public void hardL1Request() {
    switch (getState()) {
      case L1_WAIT, L1_HARD_WAIT -> setState(GroundState.L1_HARD_SCORE);
      default -> setState(GroundState.L1_WAIT);
    }
  }

  public void idleRequest() {
    if (hasCoral) {
      setState(GroundState.IDLE_CORAL);
    } else {
      setState(GroundState.IDLE_EMPTY);
    }
  }

  public void handoffWaitRequest() {
    setState(GroundState.HANDOFF_WAIT);
  }

  public void handoffReleaseRequest() {
    setState(GroundState.HANDOFF_RELEASE);
  }

  public void unjamRequest() {
    setState(GroundState.UNJAM);
  }

  public void rehomeDeployRequest() {
    setStateFromRequest(GroundState.REHOME_DEPLOY);
  }

  public void climbRequest() {
    setState(GroundState.CLIMB);
  }

  public void intakeThenHandoffRequest() {
    if (getState() == GroundState.INTAKING
        || DriverStation.isAutonomous()
        || getState() == GroundState.HANDOFF_WAIT
        || getState() == GroundState.HANDOFF_RELEASE) {
      setState(GroundState.INTAKE_THEN_HANDOFF_WAIT);
    } else {
      setState(GroundState.HANDOFF_WAIT);
    }
  }
}
