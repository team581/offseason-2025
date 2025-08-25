package frc.robot.auto_align;

import com.team581.GlobalConfig;
import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Commands;
import java.util.EnumSet;
import java.util.HashMap;

public class ReefState {
  private final EnumSet<ReefPipe> scoredL2Pipes = EnumSet.noneOf(ReefPipe.class);
  private final EnumSet<ReefPipe> scoredL3Pipes = EnumSet.noneOf(ReefPipe.class);
  private final EnumSet<ReefPipe> scoredL4Pipes = EnumSet.noneOf(ReefPipe.class);
  private final EnumSet<ReefSide> removedAlgae = EnumSet.noneOf(ReefSide.class);
  private final HashMap<ReefPipe, Integer> scoredL1Pipes = new HashMap<>();

  public ReefState() {
    if (GlobalConfig.IS_DEVELOPMENT) {
      SmartDashboard.putData("ReefState/Clear", Commands.runOnce(this::clear));
    }
  }

  public void clear() {
    scoredL1Pipes.clear();
    scoredL2Pipes.clear();
    scoredL3Pipes.clear();
    scoredL4Pipes.clear();
    removedAlgae.clear();
    DogLog.timestamp("ReefState/Clear");
    DogLog.log("ReefState/L2", scoredL2Pipes.toArray(ReefPipe[]::new));
    DogLog.log("ReefState/L3", scoredL3Pipes.toArray(ReefPipe[]::new));
    DogLog.log("ReefState/L4", scoredL4Pipes.toArray(ReefPipe[]::new));
  }

  public void removeCoral(ReefPipe pipe, ReefPipeLevel level) {
    switch (level) {
      case L1 -> {
        var currentCount = scoredL1Pipes.getOrDefault(pipe, 0);
        if (currentCount == 0) {
          return;
        } else {
          scoredL1Pipes.put(pipe, currentCount - 1);
        }
      }
      case L2 -> scoredL2Pipes.remove(pipe);
      case L3 -> scoredL3Pipes.remove(pipe);
      case L4 -> scoredL4Pipes.remove(pipe);
      default -> {}
    }
    DogLog.log("ReefState/L2", scoredL2Pipes.toArray(ReefPipe[]::new));
    DogLog.log("ReefState/L3", scoredL3Pipes.toArray(ReefPipe[]::new));
    DogLog.log("ReefState/L4", scoredL4Pipes.toArray(ReefPipe[]::new));
  }

  public void markCoralScored(ReefPipe pipe, ReefPipeLevel level) {
    switch (level) {
      case L1 -> {
        var currentCount = scoredL1Pipes.getOrDefault(pipe, 0);
        scoredL1Pipes.put(pipe, currentCount + 1);
      }
      case L2 -> scoredL2Pipes.add(pipe);
      case L3 -> scoredL3Pipes.add(pipe);
      case L4 -> scoredL4Pipes.add(pipe);
      default -> {}
    }

    DogLog.log("ReefState/L2", scoredL2Pipes.toArray(ReefPipe[]::new));
    DogLog.log("ReefState/L3", scoredL3Pipes.toArray(ReefPipe[]::new));
    DogLog.log("ReefState/L4", scoredL4Pipes.toArray(ReefPipe[]::new));
  }

  public void markAlgaeRemoved(ReefSide side) {
    removedAlgae.add(side);
    DogLog.log("ReefState/RemovedAlgae", removedAlgae.toArray(ReefSide[]::new));
  }

  public void addAlgae(ReefSide side) {
    removedAlgae.remove(side);
    DogLog.log("ReefState/RemovedAlgae", removedAlgae.toArray(ReefSide[]::new));
  }

  public boolean isCoralScored(ReefPipe pipe, ReefPipeLevel level) {
    return switch (level) {
      case L2 -> scoredL2Pipes.contains(pipe);
      case L3 -> scoredL3Pipes.contains(pipe);
      case L4 -> scoredL4Pipes.contains(pipe);
      default -> false;
    };
  }

  public boolean isAlgaeRemoved(ReefSide side) {
    return removedAlgae.contains(side);
  }

  public int getL1Count(ReefPipe pipe) {
    return scoredL1Pipes.getOrDefault(pipe, 0);
  }
}
