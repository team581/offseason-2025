package frc.robot.robot_manager;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc.robot.robot_manager.ground_manager.GroundManager;
import java.util.List;

public class RobotCommands {
  private final GroundManager groundManager;
  // private final Subsystem[] rmRequirements;
  private final Subsystem[] gmRequirements;

  // private final Subsystem[] bothRequirements;

  public RobotCommands(GroundManager groundManager) {
    this.groundManager = groundManager;
    var requirementsList =
        List.of(groundManager.deploy, groundManager.intake, groundManager.singulator);
    // rmRequirements = requirementsList.toArray(Subsystem[]::new);
    // requirementsList = List.of(robot.groundManager.deploy, robot.groundManager.intake);
    gmRequirements = requirementsList.toArray(Subsystem[]::new);
    // requirementsList =
    //     List.of(
    //         robot.elevator,
    //         robot.arm,
    //         robot.claw,
    //         robot.climber,
    //         robot.groundManager.deploy,
    //         robot.groundManager.intake);
    // bothRequirements = requirementsList.toArray(Subsystem[]::new);
  }

  public Command groundIntakeCommand() {
    return Commands.runOnce(groundManager::intakeRequest, gmRequirements)
        .withName("GroundIntakeCoralCommand");
  }

  public Command stowCommand() {
    return Commands.runOnce(groundManager::stowRequest, gmRequirements).withName("StowCommand");
  }
}
