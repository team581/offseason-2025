package frc.robot.intake_assist;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import frc.robot.config.RobotConfig;
import frc.robot.vision.game_piece_detection.GamePieceDetectionUtil;
import frc.robot.vision.results.GamePieceResult;

public final class IntakeAssistUtil {
  private static final double CORAL_ASSIST_KP = 3.0;
  private static final double INTAKE_OFFSET = Units.inchesToMeters(18);
  private static final double LOLLIPOP_INTAKE_OFFSET = Units.inchesToMeters(30);

  public static ChassisSpeeds getAssistSpeedsFromPose(
      Pose2d target, Pose2d robotPose, ChassisSpeeds teleopSpeeds) {
    var robotRelativePose =
        target
            .getTranslation()
            .minus(robotPose.getTranslation())
            .rotateBy(Rotation2d.fromDegrees(360 - robotPose.getRotation().getDegrees()));
    var sidewaysSpeed = robotRelativePose.getY();
    var forwardError = Math.hypot(teleopSpeeds.vxMetersPerSecond, teleopSpeeds.vyMetersPerSecond);
    var robotRelativeError = new Translation2d(forwardError, sidewaysSpeed * CORAL_ASSIST_KP);
    var fieldRelativeError = robotRelativeError.rotateBy(robotPose.getRotation());
    var assistSpeeds =
        new ChassisSpeeds(fieldRelativeError.getX(), fieldRelativeError.getY(), 0.0).times(0.8);
    var scaledTeleopSpeeds = teleopSpeeds.times(0.2);
    return assistSpeeds.plus(scaledTeleopSpeeds);
  }

  public static double getIntakeAssistAngle(Translation2d target, Pose2d robotPose) {
    return Units.radiansToDegrees(
        Math.atan2(target.getY() - robotPose.getY(), target.getX() - robotPose.getX()));
  }

  public static Pose2d getLollipopIntakePoseFromVisionResult(
      GamePieceResult result, Pose2d robotPose) {
    var translation =
        GamePieceDetectionUtil.calculateRobotRelativeLollipopTranslationFromCamera(
            robotPose, result);

    var fieldRelativePose =
        new Pose2d(
            translation.rotateBy(robotPose.getRotation()).plus(robotPose.getTranslation()),
            robotPose.getRotation());
    DogLog.log("CoralMap/Lollipop/RawPose", fieldRelativePose);
    var offset =
        fieldRelativePose.transformBy(
            new Transform2d(
                Units.inchesToMeters(-RobotConfig.get().arm().inchesFromCenter()),
                LOLLIPOP_INTAKE_OFFSET,
                Rotation2d.kZero));
    DogLog.log("CoralMap/Lollipop/WantedIntakePose", offset);
    return offset;
  }

  private IntakeAssistUtil() {}
}
