package frc.robot.intake;

import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.S2StateValue;
import com.team581.util.state_machines.StateMachine;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.config.RobotConfig;
import frc.robot.util.scheduling.SubsystemPriority;

public class IntakeSubsystem extends StateMachine<IntakeState> {
  private final TalonFX motor;
  private final CANdi candi;
  private final Debouncer debouncer = RobotConfig.get().intake().debouncer();

  private boolean sensorRaw = false;

  private boolean sensorDebounced = false;

  public IntakeSubsystem(TalonFX motor, CANdi candi) {
    super(SubsystemPriority.INTAKE, IntakeState.IDLE_NO_GP);

    motor.getConfigurator().apply(RobotConfig.get().intake().motorConfig());
    this.motor = motor;
    this.candi = candi;
  }

  @Override
  protected void collectInputs() {

    sensorRaw =
        candi.getS2State().getValue()
            != (RobotConfig.IS_PRACTICE_BOT ? S2StateValue.Low : S2StateValue.High);

    if (RobotBase.isSimulation()) {
      sensorRaw =
          switch (getState()) {
            case CORAL_HANDOFF, UNJAM -> !timeout(0.5);
            case IDLE_NO_GP -> false;
            case IDLE_GP -> true;
            case INTAKING -> timeout(2);
            case SCORING, HARD_SCORING -> !timeout(0.5);
          };
    }
    sensorDebounced = debouncer.calculate(sensorRaw);
  }

  public boolean getHasGP() {
    return sensorDebounced;
  }

  public void setState(IntakeState newState) {
    setStateFromRequest(newState);
  }

  @Override
  protected void afterTransition(IntakeState newState) {
    switch (newState) {
      case IDLE_NO_GP -> {
        motor.disable();
      }
      case IDLE_GP -> {
        motor.setVoltage(0);
      }
      case INTAKING -> {
        motor.setVoltage(12);
      }
      case SCORING -> {
        motor.setVoltage(-4);
      }
      case HARD_SCORING -> {
        motor.setVoltage(-5.5);
      }
      case CORAL_HANDOFF -> {
        motor.setVoltage(-12);
      }
      case UNJAM -> {
        motor.setVoltage(-6);
      }
    }
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();

    DogLog.log("Intake/Motor/AppliedVoltage", motor.getMotorVoltage().getValueAsDouble());
    DogLog.log("Intake/Motor/StatorCurrent", motor.getStatorCurrent().getValueAsDouble());
    DogLog.log("Intake/Motor/SupplyCurrent", motor.getSupplyCurrent().getValueAsDouble());
    DogLog.log("Intake/RawSensor", sensorRaw);

    DogLog.log("Intake/SensorHasGP", sensorDebounced);
  }
}
