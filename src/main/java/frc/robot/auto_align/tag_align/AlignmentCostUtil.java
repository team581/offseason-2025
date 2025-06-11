package frc.robot.auto_align.tag_align;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import frc.robot.auto_align.ReefPipe;
import frc.robot.auto_align.ReefPipeLevel;
import frc.robot.auto_align.ReefState;
import frc.robot.auto_align.RobotScoringSide;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.MathHelpers;
import java.util.Comparator;

public class AlignmentCostUtil {
  private static final double ANGLE_DIFFERENCE_SCALAR = 0.02;
  private static final double ANGLE_DIFFERENCE_SCALAR_CORAL = 0.07;

  /**
   * Returns the "cost" (a dimensionless number) of aligning to a given pose based on the robot's
   * current state.
   *
   * @param target Pose to align to
   * @param robotPose The robot's current pose
   * @param robotVelocity The robot's current velocity (field relative)
   */
  public static double getAlignCost(
      Translation2d target, Translation2d robotPose, ChassisSpeeds robotVelocity) {
    var distanceCost = target.getDistance(robotPose);
    if (target.equals(Translation2d.kZero) || robotPose.equals(Translation2d.kZero)) {
      return distanceCost;
    }

    if (Math.hypot(robotVelocity.vxMetersPerSecond, robotVelocity.vyMetersPerSecond) == 0.0) {
      return distanceCost;
    }

    var targetRobotRelative = target.minus(robotPose);
    var targetDirection = targetRobotRelative.getAngle();

    var driveAngleCost =
        ANGLE_DIFFERENCE_SCALAR
            * Math.abs(
                targetDirection.minus(MathHelpers.vectorDirection(robotVelocity)).getRadians());
    return distanceCost + driveAngleCost;
  }

  public static double getAlignCostLookahead(
      Translation2d target, Translation2d robotPose, ChassisSpeeds robotVelocity) {
    var vec = new Translation2d(robotVelocity.vxMetersPerSecond, robotVelocity.vyMetersPerSecond);
    if (vec.getNorm() == 0.0) {
      return target.getDistance(robotPose);
    }
    var distanceCost =
        target.getDistance(
            robotPose.plus(vec.times(0.02)));
                    // One loop of lookahead

    if (target.equals(Translation2d.kZero) || robotPose.equals(Translation2d.kZero)) {
      return distanceCost;
    }

    var targetRobotRelative = target.minus(robotPose);
    var targetDirection = targetRobotRelative.getAngle();

    var driveAngleCost =
        ANGLE_DIFFERENCE_SCALAR
            * Math.abs(
                targetDirection.minus(MathHelpers.vectorDirection(robotVelocity)).getRadians());
    return distanceCost + driveAngleCost;
  }

  public final Comparator<Translation2d> alignCostComparator;

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

    alignCostComparator =
        Comparator.comparingDouble(
            translation ->
                getAlignCostLookahead(
                    translation,
                    localization.getPose().getTranslation(),
                    swerve.getFieldRelativeSpeeds()));
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
            getAlignCostLookahead(
                    pipe.getPose(level, side, localization.getPose()).getTranslation(),
                    localization.getPose().getTranslation(),
                    swerve.getTeleopSpeeds())
                + (reefState.isScored(pipe, level) ? 0.4 : 0));
  }
}
