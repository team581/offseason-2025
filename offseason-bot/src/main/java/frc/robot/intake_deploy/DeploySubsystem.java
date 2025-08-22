package frc.robot.intake_deploy;

import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.team581.util.state_machines.StateMachine;
import dev.doglog.DogLog;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.util.Units;
import frc.robot.config.RobotConfig;
import frc.robot.util.scheduling.SubsystemPriority;

public class DeploySubsystem extends StateMachine<DeployState> {
  private final TalonFX motor;
  private final CoastOut coastRequest = new CoastOut();
  private final PositionVoltage positionRequest = new PositionVoltage(0.0);
  private final LinearFilter currentFilter = LinearFilter.movingAverage(6);

  final double homingVoltage = RobotConfig.get().deploy().homingVoltage();
  final double currentThreshold = RobotConfig.get().deploy().homingCurrentThreshold();
  final double endPosition = RobotConfig.get().deploy().homingEndPosition();

  private double rawCurrent = 0.0;
  private double filteredCurrent = 0.0;
  private double angle = 0.0;

  public DeploySubsystem(TalonFX motor) {
    super(SubsystemPriority.DEPLOY, DeployState.UNHOMED);

    motor.getConfigurator().apply(RobotConfig.get().deploy().motorConfig());
    this.motor = motor;
  }

  @Override
  protected DeployState getNextState(DeployState currentState) {
    return switch (currentState) {
      case REHOME -> {
        if (filteredCurrent > currentThreshold) {
          motor.setPosition(Units.degreesToRotations(endPosition));
          yield DeployState.STOWED;
        }
        yield currentState;
      }
      default -> currentState;
    };
  }

  @Override
  protected void afterTransition(DeployState newState) {
    switch (newState) {
      case UNHOMED -> motor.setControl(coastRequest);
      case REHOME -> {
        positionRequest.withPosition(endPosition);
        motor.setVoltage(homingVoltage);
      }
      default ->
          motor.setControl(positionRequest.withPosition(Units.degreesToRotations(newState.angle)));
    }
  }

  @Override
  protected void collectInputs() {
    rawCurrent = motor.getStatorCurrent().getValueAsDouble();
    filteredCurrent = currentFilter.calculate(rawCurrent);
    angle = Units.rotationsToDegrees(motor.getPosition().getValueAsDouble());
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();

    DogLog.log("Deploy/Motor/StatorCurrent", filteredCurrent);
    DogLog.log("Deploy/Angle", angle);
  }

  public void setState(DeployState newState) {
    setStateFromRequest(newState);
  }

  public void rehome() {
    if (getState() == DeployState.REHOME) {
      return;
    }
    setState(DeployState.REHOME);
  }
}
