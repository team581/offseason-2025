package frc.robot.elevator;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.controls.CoastOut;
import com.ctre.phoenix6.controls.MotionMagicVoltage;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.sim.ChassisReference;
import com.team581.math.MathHelpers;
import com.team581.util.tuning.TunablePid;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.config.FeatureFlags;
import frc.robot.config.RobotConfig;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.util.state_machines.StateMachine;

public class ElevatorSubsystem extends StateMachine<ElevatorState> {
  private static final double TOLERANCE = 0.5;
  private static final double NEAR_TOLERANCE = 20.0;

  private static double clampHeight(double height) {
    return MathUtil.clamp(
        height, RobotConfig.get().elevator().minHeight(), RobotConfig.get().elevator().maxHeight());
  }

  private final TalonFX leftMotor;
  private final TalonFX rightMotor;

  private double leftMotorCurrent;
  private double rightMotorCurrent;

  private final LinearFilter currentFilter = LinearFilter.movingAverage(5);

  private final MotionMagicVoltage positionRequest =
      new MotionMagicVoltage(ElevatorState.STOWED.getHeight());

  // Homing
  private double leftHeight = 0;
  private double rightHeight = 0;
  private double lowestSeenHeightLeft = Double.POSITIVE_INFINITY;
  private double lowestSeenHeightRight = Double.POSITIVE_INFINITY;

  private double averageMeasuredHeight = 0;
  private double collisionAvoidanceGoal = ElevatorState.STOWED.getHeight();
  // Mid-match homing
  private double averageMotorCurrent;
  private final CoastOut coastRequest = new CoastOut();

  public ElevatorSubsystem(TalonFX leftMotor, TalonFX rightMotor) {
    super(SubsystemPriority.ELEVATOR, ElevatorState.PRE_MATCH_HOMING);
    this.leftMotor = leftMotor;
    this.rightMotor = rightMotor;
    // Motor Configs
    leftMotor.getConfigurator().apply(RobotConfig.get().elevator().leftMotorConfig());
    rightMotor.getConfigurator().apply(RobotConfig.get().elevator().rightMotorConfig());

    TunablePid.of("Elevator/Left", leftMotor, RobotConfig.get().elevator().leftMotorConfig());
    TunablePid.of("Elevator/Right", rightMotor, RobotConfig.get().elevator().rightMotorConfig());
  }

  public void setState(ElevatorState newState) {
    switch (getState()) {
      case PRE_MATCH_HOMING -> {
        if (newState == ElevatorState.MID_MATCH_HOMING) {
          setStateFromRequest(newState);
        }
      }
      case MID_MATCH_HOMING -> {}
      default -> {
        setStateFromRequest(newState);
      }
    }
  }

  public double getHeight() {
    return averageMeasuredHeight;
  }

  public void setCollisionAvoidanceGoal(double height) {
    collisionAvoidanceGoal = height;
    DogLog.log("Elevator/CollisionAvoidanceGoalHeight", collisionAvoidanceGoal);
  }

  @Override
  protected void collectInputs() {
    // Calculate average height of the two motors
    leftHeight = leftMotor.getPosition().getValueAsDouble();
    rightHeight = rightMotor.getPosition().getValueAsDouble();

    averageMeasuredHeight = (leftHeight + rightHeight) / 2.0;

    leftMotorCurrent = leftMotor.getStatorCurrent().getValueAsDouble();
    rightMotorCurrent = rightMotor.getStatorCurrent().getValueAsDouble();

    averageMotorCurrent = currentFilter.calculate((leftMotorCurrent + rightMotorCurrent) / 2.0);

    if (DriverStation.isDisabled()) {
      lowestSeenHeightLeft = Math.min(lowestSeenHeightLeft, leftHeight);
      lowestSeenHeightRight = Math.min(lowestSeenHeightRight, rightHeight);
    }
  }

  @Override
  protected void afterTransition(ElevatorState newState) {
    switch (newState) {
      case MID_MATCH_HOMING -> {
        leftMotor.setVoltage(-0.0);
        rightMotor.setVoltage(-0.0);
      }
      case COLLISION_AVOIDANCE -> {
        leftMotor.setControl(positionRequest.withPosition(clampHeight(collisionAvoidanceGoal)));
        rightMotor.setControl(positionRequest.withPosition(clampHeight(collisionAvoidanceGoal)));
      }
      default -> {
        leftMotor.setControl(positionRequest.withPosition(clampHeight(newState.getHeight())));
        rightMotor.setControl(positionRequest.withPosition(clampHeight(newState.getHeight())));
      }
    }
  }

  public void customPeriodic() {
    DogLog.log("Elevator/Left/StatorCurrent", leftMotorCurrent);
    DogLog.log("Elevator/Right/StatorCurrent", rightMotorCurrent);
    DogLog.log("Elevator/AverageStatorCurrent", averageMotorCurrent);
    DogLog.log("Elevator/Left/AppliedVoltage", leftMotor.getMotorVoltage().getValueAsDouble());
    DogLog.log("Elevator/Right/AppliedVoltage", rightMotor.getMotorVoltage().getValueAsDouble());

    DogLog.log("Elevator/Left/Height", leftHeight);
    DogLog.log("Elevator/Right/Height", rightHeight);
    DogLog.log("Elevator/Height", averageMeasuredHeight);
    DogLog.log("Elevator/AtGoal", atGoal());

    switch (getState()) {
      case COLLISION_AVOIDANCE -> {
        leftMotor.setControl(positionRequest.withPosition(clampHeight(collisionAvoidanceGoal)));
        rightMotor.setControl(positionRequest.withPosition(clampHeight(collisionAvoidanceGoal)));
      }
      default -> {}
    }

    if (DriverStation.isDisabled() && FeatureFlags.FIELD_CALIBRATION.getAsBoolean()) {
      leftMotor.setControl(coastRequest);
      rightMotor.setControl(coastRequest);
    }

    var usedHeight =
        getState() == ElevatorState.COLLISION_AVOIDANCE
            ? collisionAvoidanceGoal
            : getState().getHeight();

    if (MathUtil.isNear(0, usedHeight, 0.25)
        && MathUtil.isNear(0, getHeight(), 0.25)
        && getState() != ElevatorState.MID_MATCH_HOMING) {
      leftMotor.disable();
      rightMotor.disable();
    }
  }

  @Override
  protected ElevatorState getNextState(ElevatorState currentState) {
    return switch (currentState) {
      case MID_MATCH_HOMING -> {
        if (averageMotorCurrent > RobotConfig.get().elevator().homingCurrentThreshold()) {
          leftMotor.setPosition(RobotConfig.get().elevator().homingEndHeight());
          rightMotor.setPosition(RobotConfig.get().elevator().homingEndHeight());

          // Refresh sensor data now that position is set
          collectInputs();

          yield ElevatorState.STOWED;
        }

        yield currentState;
      }

      case PRE_MATCH_HOMING -> {
        if (DriverStation.isEnabled()) {
          // We are enabled and still in pre match homing
          // Reset the motor positions, and then transition to idle state
          double homingEndHeight = RobotConfig.get().elevator().homingEndHeight();
          var leftHomedHeight = homingEndHeight + (leftHeight - lowestSeenHeightLeft);
          var rightHomedHeight = homingEndHeight + (rightHeight - lowestSeenHeightRight);
          leftMotor.setPosition(leftHomedHeight);
          rightMotor.setPosition(rightHomedHeight);

          yield ElevatorState.STOWED;
        }

        yield currentState;
      }

      default -> currentState;
    };
  }

  public boolean atGoal() {
    return switch (getState()) {
      case PRE_MATCH_HOMING, MID_MATCH_HOMING, UNJAM -> true;
      case COLLISION_AVOIDANCE -> false;
      default ->
          MathUtil.isNear(
              getState().getHeight(),
              averageMeasuredHeight,
              getState().getHeight() == 0.0 ? TOLERANCE + 1.0 : TOLERANCE);
    };
  }

  public boolean nearGoal(ElevatorState state) {
    return nearGoal(state, NEAR_TOLERANCE);
  }

  public boolean nearGoal(ElevatorState state, double tolerance) {
    return MathUtil.isNear(state.getHeight(), averageMeasuredHeight, tolerance);
  }

  public boolean nearGoal() {
    return switch (getState()) {
      case PRE_MATCH_HOMING, MID_MATCH_HOMING, UNJAM -> true;
      case COLLISION_AVOIDANCE -> false;
      default -> MathUtil.isNear(getState().getHeight(), averageMeasuredHeight, NEAR_TOLERANCE);
    };
  }

  private final TalonFXConfiguration simLeftConfig = new TalonFXConfiguration();
  private final TalonFXConfiguration simRightConfig = new TalonFXConfiguration();
  private TrapezoidProfile.Constraints simConstraints;
  private boolean simDidInit = false;

  @Override
  public void simulationPeriodic() {
    if (!simDidInit) {
      leftMotor.getConfigurator().refresh(simLeftConfig);
      rightMotor.getConfigurator().refresh(simRightConfig);

      simConstraints =
          new TrapezoidProfile.Constraints(
              MathHelpers.average(
                  simLeftConfig.MotionMagic.MotionMagicCruiseVelocity,
                  simRightConfig.MotionMagic.MotionMagicCruiseVelocity),
              MathHelpers.average(
                  simLeftConfig.MotionMagic.MotionMagicAcceleration,
                  simRightConfig.MotionMagic.MotionMagicAcceleration));

      simDidInit = true;
    }

    if (DriverStation.isDisabled()) {
      return;
    }

    var currentState =
        new TrapezoidProfile.State(
            MathHelpers.average(
                leftMotor.getPosition().getValueAsDouble(),
                rightMotor.getPosition().getValueAsDouble()),
            MathHelpers.average(
                leftMotor.getVelocity().getValueAsDouble(),
                rightMotor.getVelocity().getValueAsDouble()));
    var wantedState =
        new TrapezoidProfile.State(
            MathHelpers.average(
                leftMotor.getClosedLoopReference().getValueAsDouble(),
                rightMotor.getClosedLoopReference().getValueAsDouble()),
            0);

    var predictedState =
        new TrapezoidProfile(simConstraints).calculate(0.02, currentState, wantedState);

    var leftSim = leftMotor.getSimState();
    var rightSim = rightMotor.getSimState();

    leftSim.Orientation = ChassisReference.Clockwise_Positive;
    rightSim.Orientation = ChassisReference.CounterClockwise_Positive;

    leftSim.setRawRotorPosition(
        predictedState.position * simLeftConfig.Feedback.SensorToMechanismRatio);
    rightSim.setRawRotorPosition(
        predictedState.position * simRightConfig.Feedback.SensorToMechanismRatio);

    leftSim.setRotorVelocity(
        predictedState.velocity * simLeftConfig.Feedback.SensorToMechanismRatio);
    rightSim.setRotorVelocity(
        predictedState.velocity * simRightConfig.Feedback.SensorToMechanismRatio);
  }

  @Override
  public void disabledInit() {
    // reset position to be 0
    var leftSim = leftMotor.getSimState();
    var rightSim = rightMotor.getSimState();

    leftSim.setRawRotorPosition(0);
    rightSim.setRawRotorPosition(0);
  }
}
