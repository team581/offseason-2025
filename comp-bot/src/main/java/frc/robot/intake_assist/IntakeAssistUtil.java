package frc.robot.intake_assist;

import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
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
  private static final double ASSIST_KP = 3.7;
  private static final double INTAKE_OFFSET = Units.inchesToMeters(18);
  private static final double LOLLIPOP_INTAKE_OFFSET = Units.inchesToMeters(29.0);

  public static ChassisSpeeds getAssistSpeedsFromPose(
      Pose2d target, Pose2d robotPose, ChassisSpeeds teleopSpeeds) {
    var robotRelativePose =
        target
            .getTranslation()
            .minus(robotPose.getTranslation())
            .rotateBy(Rotation2d.fromDegrees(360 - robotPose.getRotation().getDegrees()));
    var sidewaysSpeed = MathUtil.clamp(robotRelativePose.getY() * ASSIST_KP, -1.0, 1.0);
    var robotRelativeError = new Translation2d(0, sidewaysSpeed);
    var fieldRelativeError = robotRelativeError.rotateBy(robotPose.getRotation());
    var assistSpeeds = new ChassisSpeeds(fieldRelativeError.getX(), fieldRelativeError.getY(), 0.0);

    return assistSpeeds.plus(teleopSpeeds);
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
