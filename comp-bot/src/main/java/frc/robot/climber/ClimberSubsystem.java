package frc.robot.climber;

import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.CANrange;
import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.config.RobotConfig;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;

public class ClimberSubsystem extends StateMachine<ClimberState> {
  private static final double CLIMBER_BACKWARD_VELOCITY_TRHESHOLD = 0.1;
  private static final double PASS_ANGLE_CHECK = 0.0;
  private final TalonFX climbMotor;
  private final CANcoder encoder;
  private final TalonFX grabMotor;
  private final CANrange canRange;
  private final Debouncer canRangeDebouncer = new Debouncer(0.25, DebounceType.kBoth);
  private final StaticBrake brakeNeutralRequest = new StaticBrake();
  private final LinearFilter cancoderVelocityFilter = LinearFilter.movingAverage(7);

  private final CoastOut coastNeutralRequest = new CoastOut();
  private double cancoderVelocity = 0;
  private double currentAngle = 0.0;
  private double climberMotorAngle = 0.0;
  private boolean holdingCage = false;

  public ClimberSubsystem(
      TalonFX climbMotor, CANcoder encoder, TalonFX grabMotor, CANrange canRange) {
    super(SubsystemPriority.CLIMBER, ClimberState.STOPPED);

    this.climbMotor = climbMotor;
    this.encoder = encoder;
    this.grabMotor = grabMotor;
    this.canRange = canRange;

    climbMotor.getConfigurator().apply(RobotConfig.get().climber().climbMotorConfig());
    encoder.getConfigurator().apply(RobotConfig.get().climber().cancoderConfig());
    grabMotor.getConfigurator().apply(RobotConfig.get().climber().grabMotorConfig());
    canRange.getConfigurator().apply(RobotConfig.get().climber().canRangeConfig());
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();

    switch (getState()) {
      case STOPPED -> {
        if (DriverStation.isDisabled()) {
          climbMotor.setControl(coastNeutralRequest);
        } else {
          climbMotor.disable();
        }
        grabMotor.disable();
      }
      case LINEUP_FORWARD -> {
        climbMotor.setVoltage(getState().forwardsVoltage);
        grabMotor.disable();

        if (currentAngle < PASS_ANGLE_CHECK) {
          setStateFromRequest(ClimberState.LINEUP_BACKWARD);
          DogLog.timestamp("Climber/LineupForwardStartedFlip");
        }
      }
      case LINEUP_BACKWARD -> {
        if (atGoal()) {
          climbMotor.disable();
        } else {
          climbMotor.setVoltage(getState().forwardsVoltage);
        }
        if (holdingCage) {
          setStateFromRequest(ClimberState.HANGING);
        }
        grabMotor.setVoltage(12.0);
      }
      case HANGING -> {
        if (atGoal()) {
          climbMotor.disable();
        } else {
          climbMotor.setVoltage(getState().forwardsVoltage);
        }
        grabMotor.disable();
      }
      default -> {}
    }

    if (RobotConfig.IS_DEVELOPMENT) {
      if (atGoal()) {
        DogLog.log("Climber/Status", "At goal");
      } else if (currentAngle < clamp(getState().angle)) {
        DogLog.log("Climber/Status", "Too low");
      } else {
        DogLog.log("Climber/Status", "Too high");
      }
    }
  }

  public void setState(ClimberState newState) {
    switch (newState) {
      case LINEUP_FORWARD, STOPPED -> setStateFromRequest(newState);
      case HANGING -> {
        if (getState() == ClimberState.LINEUP_BACKWARD && atGoal()) {
          setStateFromRequest(newState);
        }
      }
      default -> {}
    }
  }

  public boolean holdingCage() {
    return holdingCage;
  }

  @Override
  protected void collectInputs() {
    currentAngle = Units.rotationsToDegrees(encoder.getAbsolutePosition().getValueAsDouble());
    climberMotorAngle = Units.rotationsToDegrees(climbMotor.getPosition().getValueAsDouble());
    cancoderVelocity = cancoderVelocityFilter.calculate(encoder.getVelocity().getValueAsDouble());
    holdingCage = canRangeDebouncer.calculate(canRange.getIsDetected().getValue());

    DogLog.log("Climber/CANCoderVelocity", cancoderVelocity);

    DogLog.log("Climber/Cancoder/Angle", currentAngle);

    DogLog.log("Climber/ClimbMotor/Angle", climberMotorAngle);

    DogLog.log("Climber/HoldingCage", holdingCage);

    DogLog.log("Climber/AppliedVoltage", climbMotor.getMotorVoltage().getValueAsDouble());
  }

  public boolean atGoal() {
    var goal = clamp(getState().angle);
    return currentAngle >= goal;
  }

  private static double clamp(double angle) {
    return MathUtil.clamp(
        angle, RobotConfig.get().climber().minAngle(), RobotConfig.get().climber().maxAngle());
  }
}
