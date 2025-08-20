package frc.robot.singulator;

import com.ctre.phoenix6.hardware.TalonFX;
import com.team581.util.state_machines.StateMachine;

import dev.doglog.DogLog;
import frc.robot.config.RobotConfig;
import frc.robot.util.scheduling.SubsystemPriority;

public class SingulatorSubsystem extends StateMachine<SingulatorState> {
  private final TalonFX leftMotor;
  private final TalonFX rightMotor;
  public SingulatorSubsystem(TalonFX leftMotor, TalonFX rightMotor) {
    super(SubsystemPriority.SINGULATOR, SingulatorState.IDLE);

    leftMotor.getConfigurator().apply(RobotConfig.get().singulator().leftMotorConfig());
    rightMotor.getConfigurator().apply(RobotConfig.get().singulator().rightMotorConfig());
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
  }


  @Override
  protected void afterTransition(SingulatorState newState) {
    switch (newState) {
      default -> {
        leftMotor.setVoltage(getState().volts);
        rightMotor.setVoltage(getState().volts);
      }
      case UNTUNED -> {
        leftMotor.disable();
        rightMotor.disable();
      }
    }
  }

  @Override
  public void robotPeriodic() {
      DogLog.log("Intake/Left/Current", leftMotor.getStatorCurrent().getValueAsDouble());
      DogLog.log("Intake/Right/Current", rightMotor.getStatorCurrent().getValueAsDouble());
  }

  public void setState(SingulatorState newState) {
    setStateFromRequest(newState);
  }
}
