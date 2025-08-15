package frc.robot.autos;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import frc.robot.config.RobotConfig;
import frc.robot.util.trailblazer.AutoSegment;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

public class TrailblazerPathLogger {
  private static final Map<String, List<Pose2d>> autoNameToPath = new HashMap<>();
  private static String currentAuto = "(none)";

  public static void markAuto(BaseAuto auto) {
    currentAuto = auto.name();
  }

  public static void logSegment(AutoSegment segment) {
    if (!RobotConfig.IS_DEVELOPMENT) {
      return;
    }

    var currentPath = autoNameToPath.computeIfAbsent(currentAuto, k -> new ArrayList<>());

    currentPath.addAll(segment.points.stream().map(point -> point.poseSupplier.get()).toList());

    DogLog.log("Autos/Paths/" + currentAuto, currentPath.toArray(Pose2d[]::new));
  }

  private TrailblazerPathLogger() {}
}
