package frc.robot.lights;

import com.ctre.phoenix6.controls.SolidColor;
import com.ctre.phoenix6.controls.StrobeAnimation;
import com.ctre.phoenix6.hardware.CANdle;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.config.RobotConfig;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;

public class LightsSubsystem extends StateMachine<LightsState> {
  private final CANdle candle;
  private final SolidColor solidColorRequest = new SolidColor(0, 399).withUpdateFreqHz(50.0);
  private final StrobeAnimation blinkRequest = new StrobeAnimation(0, 399).withUpdateFreqHz(50.0);

  private LightsState storedState = LightsState.IDLE_EMPTY;
  private LightsState disabledState = LightsState.HOMED_SEES_TAGS;

  public LightsSubsystem(CANdle candle) {
    super(SubsystemPriority.LIGHTS, LightsState.IDLE_EMPTY);
    candle.getConfigurator().apply(RobotConfig.get().lights().config());
    this.candle = candle;
  }

  public void setState(LightsState newState) {
    setStateFromRequest(newState);
  }

  public void blink() {
    storedState = getState();
    setStateFromRequest(LightsState.BLINK);
  }

  public void setDisabledState(LightsState newDisabledState) {
    disabledState = newDisabledState;
  }

  @Override
  protected LightsState getNextState(LightsState currentState) {
    return switch (currentState) {
      case BLINK -> timeout(1.0) ? storedState : currentState;
      default -> currentState;
    };
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();
    var usedState = DriverStation.isDisabled() ? disabledState : getState();

    if (usedState.pattern == BlinkPattern.SOLID) {
      candle.setControl(solidColorRequest.withColor(usedState.getRGBWColor()));
    } else {
      candle.setControl(
          blinkRequest
              .withColor(usedState.getRGBWColor())
              .withFrameRate(1 / usedState.pattern.duration));
    }

    DogLog.log("Lights/Color", usedState.color.toString());
    DogLog.log("Lights/Pattern", usedState.pattern);
  }
}
