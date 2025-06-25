package frc.robot.util.kinematics;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.struct.ChassisSpeedsStruct;

public class PolarChassisSpeeds extends ChassisSpeeds {
  /** ChassisSpeeds struct for serialization. */
  public static final ChassisSpeedsStruct struct = new ChassisSpeedsStruct();

  public double vMetersPerSecond;
  public Rotation2d direction;

  public PolarChassisSpeeds(
      double vMetersPerSecond, Rotation2d direction, double omegaRadiansPerSecond) {
    super(
        vMetersPerSecond * direction.getCos(),
        vMetersPerSecond * direction.getSin(),
        omegaRadiansPerSecond);
    this.vMetersPerSecond = vMetersPerSecond;
    this.direction = direction;
  }

  public PolarChassisSpeeds(ChassisSpeeds other) {
    this(other.vxMetersPerSecond, other.vyMetersPerSecond, other.omegaRadiansPerSecond);
  }

  public PolarChassisSpeeds(double vx, double vy, double omega) {
    this(Math.hypot(vx, vy), vx == 0 && vy == 0 ? Rotation2d.kZero : new Rotation2d(vx, vy), omega);
  }

  public PolarChassisSpeeds() {
    this(0, Rotation2d.kZero, 0);
  }
}
