package frc.robot.arm;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.PositionVoltage;
import com.ctre.phoenix6.controls.StaticBrake;
import com.ctre.phoenix6.controls.VelocityVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Alert.AlertType;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotBase;
import frc.robot.config.FeatureFlags;
import frc.robot.config.RobotConfig;
import frc.robot.robot_manager.collision_avoidance.CollisionAvoidance;
import frc.robot.robot_manager.collision_avoidance.ObstructionKind;
import frc.robot.robot_manager.collision_avoidance.ObstructionStrategy;
import frc.robot.util.MathHelpers;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;
import frc.robot.util.tuning.TunablePid;
import java.util.Map;
import java.util.OptionalDouble;

public class ArmSubsystem extends StateMachine<ArmState> {
  public static final double ARM_LENGTH_METERS = Units.inchesToMeters(37.416);
  private static final double LOOKAHEADTIME = 0.02;

  private static final double TOLERANCE = 2.0;
  private static final double NEAR_TOLERANCE = 35.0;
  private static final double CLIMBER_UNSAFE_ANGLE = 225.0;
  private final TalonFX motor;
  private double rawMotorAngle;
  private double motorAngle;
  private double motorCurrent;
  private double lowestSeenAngle = Double.POSITIVE_INFINITY;
  private double highestSeenAngle = Double.NEGATIVE_INFINITY;
  private double handoffOffset = 0;
  private double collisionAvoidanceGoal;
  private static final double MINIMUM_EXPECTED_HOMING_ANGLE_CHANGE = 90.0;
  private final StaticBrake brakeNeutralRequest = new StaticBrake();
  private final CoastOut coastNeutralRequest = new CoastOut();
  private final VelocityVoltage spinToWin = new VelocityVoltage(0.6);
  private static final InterpolatingDoubleTreeMap CORAL_TX_TO_ARM_ANGLE_TABLE =
      InterpolatingDoubleTreeMap.ofEntries(
          Map.entry(2.66, 5.0), Map.entry(3.89, 0.0), Map.entry(-10.9, -5.0));

  private boolean lollipopMode = false;

  public void setLollipopMode(boolean lollipopMode) {
    this.lollipopMode = lollipopMode;
    DogLog.log("Arm/LollipopMode", lollipopMode);
  }

  private final TrapezoidProfile motionProfile =
      new TrapezoidProfile(
          new TrapezoidProfile.Constraints(
              RobotConfig.get().elevator().leftMotorConfig().MotionMagic.MotionMagicCruiseVelocity,
              RobotConfig.get().elevator().leftMotorConfig().MotionMagic.MotionMagicAcceleration));

  static TrapezoidProfile.State goalSetPoint = new TrapezoidProfile.State();
  static TrapezoidProfile.State currentSetPoint = new TrapezoidProfile.State();

  private final PositionVoltage positionRequest = new PositionVoltage(0.0).withEnableFOC(false);

  // TODO: tune velocity
  private final PositionVoltage algaeFling =
      new PositionVoltage(Units.degreesToRotations(ArmState.ALGAE_FLING_SWING.getAngle()))
          .withVelocity(Units.degreesToRotations(90));

  public ArmSubsystem(TalonFX motor) {
    super(SubsystemPriority.ARM, ArmState.PRE_MATCH_HOMING);
    motor.getConfigurator().apply(RobotConfig.get().arm().motorConfig());

    this.motor = motor;

    // In field calibration mode, boot arm to lower hardstop angle
    if (FeatureFlags.FIELD_CALIBRATION.getAsBoolean()) {
      motor.setPosition(Units.degreesToRotations(RobotConfig.get().arm().homingPosition()));
    }

    TunablePid.of("Arm", motor, RobotConfig.get().arm().motorConfig());

    DogLog.tunable(
        "Arm/ChangeEncoderPositionDeg",
        0.0,
        newPositionDeg -> motor.setPosition(Units.degreesToRotations(newPositionDeg)));
  }

  public void setCoralTx(OptionalDouble tx) {
    handoffOffset = CORAL_TX_TO_ARM_ANGLE_TABLE.get(tx.orElse(0));
  }

  public void setState(ArmState newState) {
    switch (getState()) {
      case PRE_MATCH_HOMING -> {
        if (DriverStation.isEnabled() && rangeOfMotionGood()) {
          setStateFromRequest(newState);
        }
      }
      default -> setStateFromRequest(newState);
    }
  }

  public double getAngle() {
    return motorAngle;
  }

  public double getRawAngle() {
    return rawMotorAngle;
  }

  public double getCollisionAvoidanceGoal() {
    return collisionAvoidanceGoal;
  }

  public double getArmProfileTime(double armRotations) {
    return motionProfile.timeLeftUntil(armRotations);
  }

  private void makeGetMotionMagicRequest(double armRotations) {
    goalSetPoint = new TrapezoidProfile.State(armRotations, 0);
    currentSetPoint = motionProfile.calculate(LOOKAHEADTIME, currentSetPoint, goalSetPoint);
    DogLog.log("Arm/ProfilePosition", currentSetPoint.position);
    DogLog.log("Arm/ProfileVelocity", currentSetPoint.velocity);
    // TODO: make the position request with position and velocity work
    if (DriverStation.isTeleop() || lollipopMode) {
      motor.setControl(
          positionRequest
              .withPosition(currentSetPoint.position)
              .withVelocity(currentSetPoint.velocity));
      DogLog.log("Arm/PositionVoltageStrategy", "Fast");
    } else {
      motor.setControl(
          positionRequest
              .withPosition(currentSetPoint.position)
              .withVelocity(currentSetPoint.velocity));
      DogLog.log("Arm/PositionVoaltageStrategy", "Slow");
    }
  }

  private double getSetpoint(double angle) {
    if ((Math.abs(collisionAvoidanceGoal % 360) != Math.abs(angle))
        || (Math.abs((360 - collisionAvoidanceGoal) % 360) != Math.abs(angle))) {
      return CollisionAvoidance.getCollisionAvoidanceAngleGoal(
          angle,
          true,
          ObstructionKind.NONE,
          ObstructionStrategy.IGNORE_BLOCKED,
          ObstructionStrategy.IGNORE_BLOCKED,
          rawMotorAngle);
    }
    return collisionAvoidanceGoal;
  }

  public static double getRawAngleFromNormalAngle(double angle, double rawAngle) {
    double[] solutions = CollisionAvoidance.getCollisionAvoidanceSolutions(rawAngle, angle);
    double solution1 = solutions[0];
    double solution2 = solutions[1];

    if (Math.abs(solution2 - rawAngle) > Math.abs(solution1 - rawAngle)) {
      return solution1;
    } else {
      return solution2;
    }
  }

  public void setCollisionAvoidanceGoal(double angle) {
    collisionAvoidanceGoal = angle;
    DogLog.log("Arm/CollisionAvoidanceGoalAngle", collisionAvoidanceGoal);
  }

  public boolean atGoal() {
    return switch (getState()) {
      default -> MathUtil.isNear(getState().getAngle(), rawMotorAngle, TOLERANCE, -180, 180);
      case CORAL_HANDOFF -> MathUtil.isNear(usedHandoffAngle, motorAngle, TOLERANCE, -180, 180);
      case ALGAE_FLING_SWING -> motorAngle >= getState().getAngle();
      case PRE_MATCH_HOMING, COLLISION_AVOIDANCE -> false;
    };
  }

  public boolean nearGoal() {
    return switch (getState()) {
      default -> MathUtil.isNear(getState().getAngle(), rawMotorAngle, NEAR_TOLERANCE, -180, 180);
      case PRE_MATCH_HOMING, COLLISION_AVOIDANCE -> false;
    };
  }

  public boolean nearGoal(ArmState state) {
    return nearGoal(state, NEAR_TOLERANCE);
  }

  public boolean nearGoal(ArmState state, double tolerance) {
    return MathUtil.isNear(state.getAngle(), rawMotorAngle, tolerance, -180, 180);
  }

  @Override
  protected void collectInputs() {
    usedHandoffAngle = ArmState.CORAL_HANDOFF.getAngle() + handoffOffset;
    rawMotorAngle = Units.rotationsToDegrees(motor.getPosition().getValueAsDouble());
    motorAngle = MathHelpers.angleModulus(rawMotorAngle);
    if (DriverStation.isDisabled()) {
      lowestSeenAngle = Math.min(lowestSeenAngle, rawMotorAngle);
      highestSeenAngle = Math.max(highestSeenAngle, rawMotorAngle);
    }

    motorCurrent = motor.getStatorCurrent().getValueAsDouble();
  }

  @Override
  protected void afterTransition(ArmState newState) {}

  public void customPeriodic() {
    DogLog.log("Arm/StatorCurrent", motorCurrent);
    DogLog.log("Arm/AppliedVoltage", motor.getMotorVoltage().getValueAsDouble());
    DogLog.log("Arm/Angle", motorAngle);
    DogLog.log("Arm/RawAngle", rawMotorAngle);
    DogLog.log("Arm/RangeOfMotionGood", rangeOfMotionGood());

    DogLog.log("Arm/AtGoal", atGoal());
    DogLog.log(
        "Arm/AfterHomingAngle",
        RobotConfig.get().arm().homingPosition() + (rawMotorAngle - lowestSeenAngle));

    if (DriverStation.isDisabled()) {
      DogLog.log("Arm/LowestAngle", lowestSeenAngle);
      DogLog.log("Arm/HighestAngle", highestSeenAngle);
    }
    if (rangeOfMotionGood()) {
      DogLog.clearFault("Arm not seen range of motion");
    } else if (RobotConfig.IS_PRACTICE_BOT) {
      DogLog.logFault("Arm not seen range of motion", AlertType.kWarning);
    }

    switch (getState()) {
      case COLLISION_AVOIDANCE -> {
        makeGetMotionMagicRequest(Units.degreesToRotations(collisionAvoidanceGoal));
      }
      case PRE_MATCH_HOMING -> {
        if (rangeOfMotionGood()) {
          if (DriverStation.isDisabled()) {
            motor.setControl(brakeNeutralRequest);
          }
        } else {
          motor.setControl(coastNeutralRequest);
        }
      }
      case SPIN_TO_WIN -> {
        motor.setControl(spinToWin);
      }
      case ALGAE_FLING_SWING -> {
        motor.setControl(algaeFling);
      }
      case CORAL_HANDOFF -> {
        makeGetMotionMagicRequest(Units.degreesToRotations(getSetpoint(usedHandoffAngle)));
      }
      default -> {
        makeGetMotionMagicRequest(Units.degreesToRotations(getSetpoint(getState().getAngle())));
      }
    }
  }

  @Override
  protected void beforeTransition(ArmState oldState, ArmState newState) {
    if (oldState == ArmState.PRE_MATCH_HOMING && newState != ArmState.PRE_MATCH_HOMING) {
      var actualArmAngle =
          RobotConfig.get().arm().homingPosition() + (rawMotorAngle - lowestSeenAngle);
      motor.setPosition(Units.degreesToRotations(actualArmAngle));
      // Refresh sensor data now that position is set
      collectInputs();
    }
  }

  public boolean rangeOfMotionGood() {
    return Math.abs(highestSeenAngle - lowestSeenAngle) > MINIMUM_EXPECTED_HOMING_ANGLE_CHANGE;
  }

  private final TalonFXConfiguration simMotorConfig = new TalonFXConfiguration();
  private TrapezoidProfile.Constraints simConstraints;
  private boolean simDidInit = false;

  private double usedHandoffAngle = ArmState.CORAL_HANDOFF.getAngle();

  @Override
  public void simulationPeriodic() {
    if (getState() == ArmState.PRE_MATCH_HOMING) {
      motor.setPosition(0);
      setStateFromRequest(ArmState.HOLDING_UPRIGHT);
    }

    if (!simDidInit) {
      motor.getConfigurator().refresh(simMotorConfig);

      simConstraints =
          new TrapezoidProfile.Constraints(
              simMotorConfig.MotionMagic.MotionMagicCruiseVelocity,
              simMotorConfig.MotionMagic.MotionMagicAcceleration);

      simDidInit = true;
    }

    if (DriverStation.isDisabled()) {
      return;
    }

    var currentState =
        new TrapezoidProfile.State(
            motor.getPosition().getValueAsDouble(), motor.getVelocity().getValueAsDouble());
    var wantedState =
        new TrapezoidProfile.State(motor.getClosedLoopReference().getValueAsDouble(), 0);

    var predictedState =
        new TrapezoidProfile(simConstraints).calculate(0.02, currentState, wantedState);

    var motorSim = motor.getSimState();

    motorSim.setRawRotorPosition(
        predictedState.position * simMotorConfig.Feedback.SensorToMechanismRatio);

    motorSim.setRotorVelocity(
        predictedState.velocity * simMotorConfig.Feedback.SensorToMechanismRatio);
  }

  @Override
  public void disabledInit() {
    if (RobotBase.isSimulation()) {
      // reset position to be 0*
      var motorSim = motor.getSimState();
      motorSim.setRawRotorPosition(getRawAngleFromNormalAngle(0, rawMotorAngle));
    }
  }
}
