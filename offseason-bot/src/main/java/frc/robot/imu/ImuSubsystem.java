package frc.robot.imu;

import com.ctre.phoenix6.hardware.Pigeon2;
import com.team581.util.state_machines.StateMachine;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import frc.robot.util.scheduling.SubsystemPriority;

public class ImuSubsystem extends StateMachine<ImuState> {
  private static final double IS_TILTED_THRESHOLD = 4.0;
  private static final Debouncer IS_TILTED_DEBOUNCE = new Debouncer(0.5, DebounceType.kRising);
  private final Pigeon2 imu;
  private double robotHeading = 0;
  private double pitch;
  private double angularVelocity;
  private double pitchRate;
  private double roll;
  private double rollRate;

  public ImuSubsystem(Pigeon2 imu) {
    super(SubsystemPriority.IMU, ImuState.DEFAULT_STATE);
    this.imu = imu;
  }

  @Override
  protected void collectInputs() {
    robotHeading = MathUtil.inputModulus(imu.getYaw().getValueAsDouble(), -180, 180);
    angularVelocity = imu.getAngularVelocityZWorld().getValueAsDouble();
    pitch = imu.getPitch().getValueAsDouble();
    pitchRate = imu.getAngularVelocityYWorld().getValueAsDouble();
    roll = imu.getRoll().getValueAsDouble();
    rollRate = imu.getAngularVelocityXWorld().getValueAsDouble();
  }

  public double getRobotHeading() {
    return robotHeading;
  }

  public double getRobotAngularVelocity() {
    return angularVelocity;
  }

  public double getPitch() {
    return pitch;
  }

  public double getPitchRate() {
    return pitchRate;
  }

  public double getRoll() {
    return roll;
  }

  public double getRollRate() {
    return rollRate;
  }

  public boolean isFlatDebounced() {
    return IS_TILTED_DEBOUNCE.calculate(
        MathUtil.isNear(pitch, 0, IS_TILTED_THRESHOLD)
            && MathUtil.isNear(roll, 0, IS_TILTED_THRESHOLD));
  }

  public void setAngle(double zeroAngle) {
    this.imu.setYaw(zeroAngle);
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();
    DogLog.log("Imu/RobotHeading", robotHeading);
    DogLog.log("Imu/AngularVelocity", angularVelocity);
    DogLog.log("Imu/RollVelocity", rollRate);
    DogLog.log("Imu/Roll", roll);
    DogLog.log("Imu/PitchVelocity", pitchRate);
    DogLog.log("Imu/Pitch", pitch);
  }
}
