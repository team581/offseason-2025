package frc.robot.claw;

import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.signals.S1StateValue;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.config.RobotConfig;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;

public class ClawSubsystem extends StateMachine<ClawState> {
  private final TalonFX motor;
  private final CANdi candi;
  private final Debouncer debouncer = RobotConfig.get().claw().debouncer();

  private boolean sensorRaw = false;
  private boolean sensorDebounced = false;

  public ClawSubsystem(TalonFX motor, CANdi candi) {
    super(SubsystemPriority.INTAKE, ClawState.IDLE_NO_GP);

    motor.getConfigurator().apply(RobotConfig.get().claw().motorConfig());
    this.motor = motor;
    this.candi = candi;
  }

  @Override
  protected void collectInputs() {
    var candiValue = candi.getS1State().getValue();
    sensorRaw =
        RobotConfig.get().claw().sensorFlipped()
            ? candiValue == S1StateValue.Low
            : candiValue != S1StateValue.Low;

    if (RobotBase.isSimulation()) {
      sensorRaw =
          switch (getState()) {
            case CORAL_HANDOFF -> timeout(0.5);
            case IDLE_NO_GP -> false;
            case IDLE_W_ALGAE, IDLE_W_CORAL -> true;
            case INTAKING_ALGAE -> timeout(1);
            case SCORE_CORAL -> timeout(1);
            case SCORE_ALGAE_NET, SCORE_ALGAE_PROCESSOR, OUTTAKING -> timeout(0.25);
            case LOLLIPOP_CORAL_INTAKE -> timeout(1.7);
          };
    }

    sensorDebounced = debouncer.calculate(sensorRaw);
  }

  public boolean getHasGP() {
    return sensorDebounced;
  }

  public void setState(ClawState newState) {
    setStateFromRequest(newState);
  }

  @Override
  protected void afterTransition(ClawState newState) {
    switch (newState) {
      case IDLE_NO_GP -> {
        motor.disable();
      }
      case IDLE_W_ALGAE -> {
        motor.setVoltage(12.0);
      }
      case IDLE_W_CORAL -> {
        motor.setVoltage(0.6);
      }
      case INTAKING_ALGAE -> {
        motor.setVoltage(12.0);
      }
      case CORAL_HANDOFF -> {
        motor.setVoltage(12.0);
      }
      case SCORE_ALGAE_NET -> {
        motor.setVoltage(-8);
      }
      case SCORE_ALGAE_PROCESSOR -> {
        motor.setVoltage(-5);
      }
      case SCORE_CORAL -> {
        motor.setVoltage(-4);
      }
      case OUTTAKING -> {
        motor.setVoltage(-6);
      }
      case LOLLIPOP_CORAL_INTAKE -> {
        motor.setVoltage(12);
      }
    }
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();

    DogLog.log("Claw/Motor/AppliedVoltage", motor.getMotorVoltage().getValueAsDouble());
    DogLog.log("Claw/Motor/StatorCurrent", motor.getStatorCurrent().getValueAsDouble());
    DogLog.log("Claw/Motor/SupplyCurrent", motor.getSupplyCurrent().getValueAsDouble());
    DogLog.log("Claw/Sensors/SensorRaw", sensorRaw);
    DogLog.log("Claw/Sensors/SensorDebounced", sensorDebounced);
  }
}
