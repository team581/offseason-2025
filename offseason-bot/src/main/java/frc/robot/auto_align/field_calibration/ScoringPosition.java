package frc.robot.auto_align.field_calibration;

import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.auto_align.ReefPipe;
import frc.robot.auto_align.ReefPipeLevel;

public record ScoringPosition(ReefPipe pipe, boolean isRedAlliance) {
  public Pose2d getPose(ReefPipeLevel level) {
    return pipe.getPose(level, isRedAlliance);
  }
}
