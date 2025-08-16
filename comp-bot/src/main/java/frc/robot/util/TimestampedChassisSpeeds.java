package frc.robot.util;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;

public class TimestampedChassisSpeeds extends ChassisSpeeds {
  public final double timestampSeconds;

  public TimestampedChassisSpeeds(double vx, double vy, double omega, double timestampSeconds) {
    super(vx, vy, omega);
    this.timestampSeconds = timestampSeconds;
  }

  public TimestampedChassisSpeeds(double vx, double vy, double omega) {
    this(vx, vy, omega, Timer.getFPGATimestamp());
  }

  public TimestampedChassisSpeeds(ChassisSpeeds speeds, double timestampSeconds) {
    this(
        speeds.vxMetersPerSecond,
        speeds.vyMetersPerSecond,
        speeds.omegaRadiansPerSecond,
        timestampSeconds);
  }

  public TimestampedChassisSpeeds(ChassisSpeeds speeds) {
    this(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond, speeds.omegaRadiansPerSecond);
  }

  public TimestampedChassisSpeeds(TimestampedChassisSpeeds speeds) {
    this(
        speeds.vxMetersPerSecond,
        speeds.vyMetersPerSecond,
        speeds.omegaRadiansPerSecond,
        speeds.timestampSeconds);
  }

  public TimestampedChassisSpeeds(double timestampSeconds) {
    this(0, 0, 0, timestampSeconds);
  }

  public TimestampedChassisSpeeds() {
    this(Timer.getFPGATimestamp());
  }

  public double timestampDifference(TimestampedChassisSpeeds other) {
    return timestampSeconds - other.timestampSeconds;
  }
}
