package frc.robot.autos;

import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.arm.ArmState;
import frc.robot.auto_align.ReefPipe;
import frc.robot.auto_align.RobotScoringSide;
import frc.robot.elevator.ElevatorState;
import frc.robot.robot_manager.RobotCommands;
import frc.robot.robot_manager.RobotManager;
import frc.robot.robot_manager.RobotState;
import frc.robot.robot_manager.ground_manager.GroundState;

public class AutoCommands {
  private final RobotCommands robotCommands;
  private final RobotManager robotManager;

  public AutoCommands(RobotCommands robotCommands, RobotManager robotManager) {
    this.robotCommands = robotCommands;
    this.robotManager = robotManager;
  }

  public Command stowRequest() {
    return Commands.runOnce(robotManager::stowRequest).withName("StowRequestCommand");
  }

  public Command preloadCoralCommand() {
    return Commands.runOnce(robotManager::preloadCoralRequest).withName("PreloadCoralCommand");
  }

  public boolean alignedForScore() {
    return robotManager.autoAlign.isTagAlignedDebounced()
        && robotManager.imu.isFlatDebounced()
        && robotManager.elevator.atGoal()
        && robotManager.arm.atGoal();
  }

  public Command intakeCoralHorizontalCommand() {
    return Commands.runOnce(robotManager.groundManager::intakeRequest)
        .withName("IntakeCoralHorizontalCommand");
  }

  public Command lollipopApproachCommand() {
    return Commands.runOnce(robotManager::lollipopIntakeApproachRequest)
        .withName("LollipopApproachCommand");
  }

  public Command intakeLollipopCommand() {
    return Commands.runOnce(robotManager::lollipopIntakeGrabRequest)
        .withName("LollipopIntakeCommand");
  }

  public Command homeDeployCommand() {
    return Commands.runOnce(robotManager.groundManager::rehomeDeployRequest)
        .withName("HomeDeployCommand");
  }

  public Command waitForIntakeDone() {
    if (RobotBase.isSimulation()) {
      return Commands.waitSeconds(1.0);
    }
    return robotManager
        .groundManager
        .waitForStates(GroundState.HANDOFF_WAIT)
        .withName("WaitForIntakeDoneCommand");
  }

  public Command waitForLollipopIntakeDone() {
    return robotManager
        .waitForStates(RobotState.CORAL_INTAKE_LOLLIPOP_PUSH, RobotState.CLAW_CORAL)
        .withTimeout(3.0)
        .withName("WaitForLollipopIntakeDoneCommand");
  }

  public Command waitForElevatorAndArmNearLollipop() {
    return Commands.waitUntil(
            () ->
                robotManager.elevator.nearGoal(ElevatorState.LOLLIPOP_CORAL_INTAKE_INTAKE)
                    && robotManager.arm.nearGoal(ArmState.LOLLIPOP_CORAL_INTAKE_INTAKE, 50))
        .withName("WaitForElevatorAndArmNearGoalCommand");
  }

  public Command l4ApproachCommand(ReefPipe pipe, RobotScoringSide scoringSide) {
    return Commands.runOnce(
            () -> {
              robotManager.autoAlign.setAutoReefPipeOverride(pipe);
              if (scoringSide == RobotScoringSide.LEFT) {
                robotManager.l4CoralLeftAutoApproachRequest();
              } else {
                robotManager.l4CoralRightAutoApproachRequest();
              }
            })
        .withName("L4ApproachCommand");
  }

  public Command l3ApproachCommand(RobotScoringSide scoringSide) {
    return Commands.runOnce(
            () -> {
              if (scoringSide == RobotScoringSide.LEFT) {
                robotManager.l3CoralLeftAutoApproachRequest();
              } else {
                robotManager.l3CoralRightAutoApproachRequest();
              }
            })
        .withName("L3ApproachCommand");
  }

  public Command l2ApproachCommand(ReefPipe pipe, RobotScoringSide scoringSide) {
    return Commands.runOnce(
            () -> {
              robotManager.autoAlign.setAutoReefPipeOverride(pipe);
              if (scoringSide == RobotScoringSide.LEFT) {
                robotManager.l2CoralLeftAutoApproachRequest();
              } else {
                robotManager.l2CoralRightAutoApproachRequest();
              }
            })
        .withName("L2ApproachCommand");
  }

  public Command l2LineupCommand(RobotScoringSide scoringSide) {
    return Commands.runOnce(
            () -> {
              if (scoringSide == RobotScoringSide.LEFT) {
                robotManager.l2CoralLeftAutoLineupRequest();
              } else {
                robotManager.l2CoralRightAutoLineupRequest();
              }
            })
        .withName("L2LineupCommand");
  }

  public Command l4LeftReleaseCommand(ReefPipe pipe, RobotScoringSide scoringSide) {
    return Commands.runOnce(
            () -> {
              robotManager.autoAlign.setAutoReefPipeOverride(pipe);
              robotManager.l4CoralLeftReleaseRequest();
            })
        .withName("L4LeftReleaseCommand");
  }

  public Command l3LeftReleaseCommand() {
    return Commands.runOnce(robotManager::l3CoralLeftReleaseRequest).withName("L3ReleaseCommand");
  }

  public Command moveToStartingPositionCommand() {
    return Commands.runOnce(robotManager::startingPositionRequest)
        .withName("MoveToStartingPositionCommand");
  }

  public Command waitForReleaseCommand() {
    return robotManager
        .waitForStates(
            RobotState.CORAL_L2_LEFT_RELEASE,
            RobotState.CORAL_L4_LEFT_RELEASE,
            RobotState.CORAL_L3_LEFT_RELEASE,
            RobotState.CORAL_L2_RIGHT_RELEASE,
            RobotState.CORAL_L3_RIGHT_RELEASE,
            RobotState.CORAL_L4_RIGHT_RELEASE)
        .withName("WaitForReleaseCommand");
  }

  public Command waitForAlignedForScore() {
    if (RobotBase.isSimulation()) {
      return Commands.waitSeconds(1.0);
    }
    return Commands.waitUntil(this::alignedForScore)
        .withTimeout(5)
        .withName("WaitForAlignedForScore");
  }

  public Command groundIntakeToL4Command() {
    return Commands.runOnce(
            () -> {
              robotManager.groundManager.intakeThenHandoffRequest();
              robotManager.l4CoralApproachRequest();
            })
        .withName("GroundIntakeL4Command");
  }
}
