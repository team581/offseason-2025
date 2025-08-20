package frc.robot.intake;

import com.ctre.phoenix6.hardware.TalonFX;
import com.team581.util.state_machines.StateMachine;

import dev.doglog.DogLog;
import frc.robot.config.RobotConfig;
import frc.robot.util.scheduling.SubsystemPriority;

public class IntakeSubsystem extends StateMachine<IntakeState> {
  private final TalonFX motor;

  public IntakeSubsystem(TalonFX motor) {
    super(SubsystemPriority.INTAKE, IntakeState.IDLE);

    motor.getConfigurator().apply(RobotConfig.get().intake().motorConfig());
    this.motor = motor;
  }

  @Override
  protected void afterTransition(IntakeState newState) {
    switch (newState) {
      default -> motor.setVoltage(getState().volts);
      case UNTUNED, STOPPED -> motor.disable();
    }
  }

  @Override
  public void robotPeriodic() {
      DogLog.log("Intake/Motor/Current", motor.getStatorCurrent().getValueAsDouble());
  }

  public void setState(IntakeState newState) {
    setStateFromRequest(newState);
  }
}
