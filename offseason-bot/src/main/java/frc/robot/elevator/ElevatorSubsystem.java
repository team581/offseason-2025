package frc.robot.elevator;

import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team581.util.state_machines.StateMachine;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.config.RobotConfig;
import frc.robot.util.scheduling.SubsystemPriority;

public class ElevatorSubsystem extends StateMachine<ElevatorState> {
  private final double TOLERANCE = 5.0;
  private final double NEAR_TOLERANCE = 20.0;

  private final TalonFX motor;
  private double height = 0.0;

  private final LinearFilter currentFilter = LinearFilter.movingAverage(5);
  private double filteredCurrent = 0.0;
  final double homingEndHeight = RobotConfig.get().elevator().homingEndHeight();

  private final MotionMagicVoltage positionRequest = new MotionMagicVoltage(ElevatorState.STOWED.height);

  public ElevatorSubsystem(TalonFX motor) {
    super(SubsystemPriority.ELEVATOR, ElevatorState.STOWED);

    motor.getConfigurator().apply(RobotConfig.get().elevator().motorConfig());
    this.motor = motor;
  }

  private double clampHeight(double unclamped) {
    return MathUtil.clamp(unclamped, RobotConfig.get().elevator().minHeight(), RobotConfig.get().elevator().maxHeight());
  }

  @Override
  protected ElevatorState getNextState(ElevatorState currentState) {
      return switch (currentState) {
        case REHOME -> {
          if (filteredCurrent > RobotConfig.get().elevator().homingCurrentThreshold()) {
            motor.setPosition(homingEndHeight);
            yield ElevatorState.STOWED;
          }
          yield currentState;
        }
        default -> currentState;
      };
  }

  @Override
  protected void afterTransition(ElevatorState newState) {
      switch (newState) {
        case REHOME -> motor.setVoltage(RobotConfig.get().elevator().homingVoltage());
        default -> motor.setControl(positionRequest.withPosition(clampHeight(newState.height)));
      }
  }

  @Override
  public void robotPeriodic() {
      DogLog.log("Elevator/Motor/Current", filteredCurrent);
      DogLog.log("Elevator/Height", height);

      if (DriverStation.isEnabled()) {
       return;
      }
      if (height < homingEndHeight) {
        motor.setPosition(homingEndHeight);
      }
  }

  @Override
  protected void collectInputs() {
    var rawCurrent = motor.getStatorCurrent().getValueAsDouble();
    filteredCurrent = currentFilter.calculate(rawCurrent);

    height = motor.getPosition().getValueAsDouble();
  }

  public void rehome() {
    setState(ElevatorState.REHOME);
  }

  public void setState(ElevatorState newState) {
    if (getState() != ElevatorState.REHOME)
    setStateFromRequest(newState);
  }

  public boolean atGoal() {
    return switch (getState()) {
      case UNJAM -> true;
      case UNTUNED, REHOME -> false;
      default -> MathUtil.isNear(clampHeight(getState().height), height, TOLERANCE);
    };
  }

  public boolean nearGoal() {
    return switch (getState()) {
      case UNJAM -> true;
      case UNTUNED, REHOME -> false;
      default -> MathUtil.isNear(clampHeight(getState().height), height, NEAR_TOLERANCE);
    };
  }

  public double getHeight() {
    return height;
  }
}
