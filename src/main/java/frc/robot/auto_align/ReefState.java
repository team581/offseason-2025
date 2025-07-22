package frc.robot.auto_align;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.config.RobotConfig;
import java.util.EnumSet;

public class ReefState {
  private final EnumSet<ReefPipe> scoredL2Pipes = EnumSet.noneOf(ReefPipe.class);
  private final EnumSet<ReefPipe> scoredL3Pipes = EnumSet.noneOf(ReefPipe.class);
  private final EnumSet<ReefPipe> scoredL4Pipes = EnumSet.noneOf(ReefPipe.class);

  public ReefState() {
    if (RobotConfig.IS_DEVELOPMENT) {
      SmartDashboard.putData("ReefState/Clear", Commands.runOnce(this::clear));
    }
  }

  public void clear() {
    scoredL2Pipes.clear();
    scoredL3Pipes.clear();
    scoredL4Pipes.clear();
    DogLog.timestamp("ReefState/Clear");
    DogLog.log("ReefState/L2", scoredL2Pipes.toArray(ReefPipe[]::new));
    DogLog.log("ReefState/L3", scoredL3Pipes.toArray(ReefPipe[]::new));
    DogLog.log("ReefState/L4", scoredL4Pipes.toArray(ReefPipe[]::new));
  }

  public void remove(ReefPipe pipe, ReefPipeLevel level) {
    switch (level) {
      case L2 -> scoredL2Pipes.remove(pipe);
      case L3 -> scoredL3Pipes.remove(pipe);
      case L4 -> scoredL4Pipes.remove(pipe);
      default -> {}
    }
  }
  public void markScored(ReefPipe pipe, ReefPipeLevel level) {
    switch (level) {
      case L2 -> scoredL2Pipes.add(pipe);
      case L3 -> scoredL3Pipes.add(pipe);
      case L4 -> scoredL4Pipes.add(pipe);
      default -> {}
    }

    DogLog.log("ReefState/L2", scoredL2Pipes.toArray(ReefPipe[]::new));
    DogLog.log("ReefState/L3", scoredL3Pipes.toArray(ReefPipe[]::new));
    DogLog.log("ReefState/L4", scoredL4Pipes.toArray(ReefPipe[]::new));
  }

  public boolean isScored(ReefPipe pipe, ReefPipeLevel level) {
    return switch (level) {
      case L2 -> scoredL2Pipes.contains(pipe);
      case L3 -> scoredL3Pipes.contains(pipe);
      case L4 -> scoredL4Pipes.contains(pipe);
      default -> false;
    };
  }
}
