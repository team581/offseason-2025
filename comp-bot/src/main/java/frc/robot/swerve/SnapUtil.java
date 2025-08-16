package frc.robot.swerve;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.auto_align.ReefSide;
import frc.robot.auto_align.RobotScoringSide;
import frc.robot.fms.FmsSubsystem;
import java.util.Comparator;
import java.util.stream.Stream;

public class SnapUtil {
  public static double getProcessorAngle() {
    return FmsSubsystem.isRedAlliance() ? 190 : 370;
  }

  public static double getCageAngle(boolean isRedAlliance) {
    return isRedAlliance ? 90 : 270;
  }

  public static double getCageAngle(Pose2d robotPose) {
    return getCageAngle(robotPose.getX() > (17.55 / 2.0));
  }

  public static double getNetScoringAngle(RobotScoringSide scoringSide, Pose2d robotPose) {
    double robotX = robotPose.getX();
    // entire field length is 17.55m
    double halfFieldLength = 17.55 / 2.0;

    if (robotX < halfFieldLength) {

      return scoringSide.equals(RobotScoringSide.RIGHT) ? 90 : -90;
    }
    return scoringSide.equals(RobotScoringSide.RIGHT) ? -90 : 90;
  }

  public static double getCoralStationAngle(Pose2d robotPose) {
    if (robotPose.getY() > 4.025) {
      if (FmsSubsystem.isRedAlliance()) {
        // Coral station red, processor side
        return 234.0;
      }

      // Coral station blue, non processor side
      return 306.0;
    } else {
      if (FmsSubsystem.isRedAlliance()) {
        // Coral station red, non processor side
        return 126.0;
      }
      // Coral station blue, processor side
      return 54.0;
    }
  }

  public static double getNearestReefAngle(Pose2d robotPose) {
    return Stream.of(ReefSide.values())
        .min(
            Comparator.comparingDouble(
                side ->
                    side.getPose(robotPose)
                        .getTranslation()
                        .getDistance(robotPose.getTranslation())))
        .orElseThrow()
        .getPose(robotPose)
        .getRotation()
        .getDegrees();
  }

  private SnapUtil() {}
}
