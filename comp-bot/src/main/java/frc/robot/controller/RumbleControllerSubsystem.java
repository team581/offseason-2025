package frc.robot.controller;

import com.team581.util.state_machines.StateMachine;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.util.scheduling.SubsystemPriority;

public class RumbleControllerSubsystem extends StateMachine<RumbleControllerState> {
  private final Timer matchTimer = new Timer();
  private final GenericHID controller;
  private final boolean matchTimeRumble;
  public static final double MATCH_DURATION_TELEOP = 135;

  @Override
  public void teleopInit() {
    matchTimer.reset();
    matchTimer.start();
  }

  @Override
  public void disabledInit() {
    matchTimer.stop();
  }

  public RumbleControllerSubsystem(CommandXboxController controller, boolean matchTimeRumble) {
    this(controller.getHID(), matchTimeRumble);

    if (matchTimeRumble) {
      var rumbleCommand = Commands.runOnce(() -> rumbleRequest());
      new Trigger(() -> matchTimer.hasElapsed(MATCH_DURATION_TELEOP - 90)).onTrue(rumbleCommand);
      new Trigger(() -> matchTimer.hasElapsed(MATCH_DURATION_TELEOP - 60)).onTrue(rumbleCommand);
      new Trigger(() -> matchTimer.hasElapsed(MATCH_DURATION_TELEOP - 30)).onTrue(rumbleCommand);
    }
  }

  public RumbleControllerSubsystem(GenericHID controller, boolean matchTimeRumble) {
    super(SubsystemPriority.RUMBLE_CONTROLLER, RumbleControllerState.OFF);
    this.controller = controller;
    this.matchTimeRumble = matchTimeRumble;
  }

  public void rumbleRequest() {
    if (!DriverStation.isAutonomous()) {
      setStateFromRequest(RumbleControllerState.ON);
    }
  }

  @Override
  protected RumbleControllerState getNextState(RumbleControllerState currentState) {
    return switch (currentState) {
      case ON -> timeout(0.5) ? RumbleControllerState.OFF : currentState;
      case OFF -> currentState;
    };
  }

  @Override
  protected void afterTransition(RumbleControllerState newState) {
    switch (newState) {
      case ON -> {
        controller.setRumble(RumbleType.kBothRumble, 1);
      }
      case OFF -> {
        controller.setRumble(RumbleType.kBothRumble, 0);
      }
    }
  }
}
