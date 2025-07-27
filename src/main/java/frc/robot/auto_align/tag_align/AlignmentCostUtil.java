package frc.robot.auto_align.tag_align;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.auto_align.ReefPipe;
import frc.robot.auto_align.ReefPipeLevel;
import frc.robot.auto_align.ReefState;
import frc.robot.auto_align.RobotScoringSide;
import frc.robot.config.FeatureFlags;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.MathHelpers;
import java.util.Comparator;

public class AlignmentCostUtil {
  private static final double REEF_STATE_COST = 1.2;
  private static final double DRIVE_DIRECTION_SCALAR = 0.02;
  private static final double ANGLE_ERROR_SCALAR = 0.3;

  private static final double DRIVE_DIRECTION_SCALAR_CORAL = 0.07;

  /**
   * Returns the "cost" (a dimensionless number) of aligning to a given pose based on the robot's
   * current state.
   *
   * @param target Pose to align to
   * @param robotPose The robot's current pose
   * @param robotVelocity The robot's current velocity (field relative)
   */
  public static double getAlignCost(Pose2d target, Pose2d robotPose, ChassisSpeeds robotVelocity) {
    var distanceCost =
        FeatureFlags.AUTO_ALIGN_DISTANCE_COST.getAsBoolean()
            ? target.getTranslation().getDistance(robotPose.getTranslation())
            : 0.0;
    if (target.equals(Pose2d.kZero) || robotPose.equals(Pose2d.kZero)) {
      return distanceCost;
    }

    var angleToAim = target.getRotation().getRadians();

    var angleError =
        Math.abs(MathUtil.angleModulus(angleToAim - robotPose.getRotation().getRadians()));

    var angleErrorCost =
        FeatureFlags.AUTO_ALIGN_HEADING_COST.getAsBoolean() ? angleError * ANGLE_ERROR_SCALAR : 0.0;

    if (Math.hypot(robotVelocity.vxMetersPerSecond, robotVelocity.vyMetersPerSecond) == 0.0) {
      return distanceCost + angleErrorCost;
    }

    var targetRobotRelative = target.minus(robotPose);
    var targetDirection = targetRobotRelative.getTranslation().getAngle();

    var driveAngleCost =
        FeatureFlags.AUTO_ALIGN_DRIVE_DIRECTION_COST.getAsBoolean()
            ? DRIVE_DIRECTION_SCALAR
                * Math.abs(
                    targetDirection.minus(MathHelpers.vectorDirection(robotVelocity)).getRadians())
            : 0.0;
    return distanceCost + angleErrorCost + driveAngleCost;
  }

  public static double getCoralAlignCost(
      Pose2d target, Pose2d robotPose, ChassisSpeeds robotVelocity) {

    var distanceCost = target.getTranslation().getDistance(robotPose.getTranslation());
    if (target.getTranslation().equals(Translation2d.kZero)
        || robotPose.getTranslation().equals(Translation2d.kZero)) {
      return distanceCost;
    }

    if (Math.hypot(robotVelocity.vxMetersPerSecond, robotVelocity.vyMetersPerSecond) == 0.0) {
      return distanceCost;
    }

    var targetRobotRelative = target.getTranslation().minus(robotPose.getTranslation());
    var targetDirection = targetRobotRelative.getAngle();

    var driveAngleCost =
        DRIVE_DIRECTION_SCALAR_CORAL
            * Math.abs(
                targetDirection.minus(MathHelpers.vectorDirection(robotVelocity)).getRadians());
    return distanceCost + driveAngleCost;
  }

  private final LocalizationSubsystem localization;
  private final SwerveSubsystem swerve;
  private final ReefState reefState;
  private RobotScoringSide side;

  private final Comparator<ReefPipe> pipeL4Comparator = createReefPipeComparator(ReefPipeLevel.L4);
  private final Comparator<ReefPipe> pipeL3Comparator = createReefPipeComparator(ReefPipeLevel.L3);
  private final Comparator<ReefPipe> pipeL2Comparator = createReefPipeComparator(ReefPipeLevel.L2);

  public AlignmentCostUtil(
      LocalizationSubsystem localization,
      SwerveSubsystem swerve,
      ReefState reefState,
      RobotScoringSide side) {
    this.localization = localization;
    this.swerve = swerve;
    this.reefState = reefState;
    this.side = side;
  }

  public Comparator<ReefPipe> getReefPipeComparator(ReefPipeLevel level) {
    return switch (level) {
      case L4 -> pipeL4Comparator;
      case L3 -> pipeL3Comparator;
      case L2 -> pipeL2Comparator;
      // Shouldn't ever happen
      default -> pipeL2Comparator;
    };
  }

  public void setSide(RobotScoringSide side) {
    this.side = side;
  }

  /** Helper function to create a singleton comparator for each level. */
  private Comparator<ReefPipe> createReefPipeComparator(ReefPipeLevel level) {
    return Comparator.comparingDouble(
        pipe ->
            getAlignCost(
                    pipe.getPose(level, side, localization.getPose()),
                    localization.getPose(),
                    swerve.getTeleopSpeeds())
                + (reefState.isScored(pipe, level)
                        && FeatureFlags.AUTO_ALIGN_REEF_STATE_COST.getAsBoolean()
                    ? REEF_STATE_COST
                    : 0.0));
  }
}
