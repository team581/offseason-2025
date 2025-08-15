package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;

public record PoseErrorTolerance(double linearErrorTolerance, double angularErrorTolerance) {
  public PoseErrorTolerance(double linearErrorTolerance, Rotation2d angularErrorTolerance) {
    this(linearErrorTolerance, angularErrorTolerance.getDegrees());
  }

  public boolean atPose(Pose2d expected, Pose2d actual) {
    var linearError = expected.getTranslation().getDistance(actual.getTranslation());

    // Linear error within tolerance
    return MathUtil.isNear(0, linearError, linearErrorTolerance)
        &&
        // Rotation error within tolerance
        MathUtil.isNear(
            expected.getRotation().getDegrees(),
            actual.getRotation().getDegrees(),
            angularErrorTolerance,
            -180,
            180);
  }
}
