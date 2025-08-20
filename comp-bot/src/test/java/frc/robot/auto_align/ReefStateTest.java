package frc.robot.auto_align;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import java.util.List;
import org.junit.jupiter.api.Test;

public class ReefStateTest {
  private static final List<ReefPipeLevel> TRACKED_LEVELS =
      List.of(ReefPipeLevel.L2, ReefPipeLevel.L3, ReefPipeLevel.L4);

  @Test
  public void markScoredTest() {
    var state = new ReefState();

    assertFalse(state.isScored(ReefPipe.PIPE_A, ReefPipeLevel.L4));

    state.markScored(ReefPipe.PIPE_A, ReefPipeLevel.L4);

    for (var pipe : ReefPipe.values()) {
      for (var level : TRACKED_LEVELS) {
        var result = state.isScored(pipe, level);
        if (pipe == ReefPipe.PIPE_A && level == ReefPipeLevel.L4) {
          assertTrue(result);
        } else {
          assertFalse(result);
        }
      }
    }
  }

  @Test
  public void initialStateTest() {
    var state = new ReefState();

    for (var pipe : ReefPipe.values()) {
      for (var level : TRACKED_LEVELS) {
        assertFalse(state.isScored(pipe, level));
      }
    }
  }

  @Test
  public void clearTest() {
    var state = new ReefState();

    state.markScored(ReefPipe.PIPE_A, ReefPipeLevel.L4);
    state.markScored(ReefPipe.PIPE_B, ReefPipeLevel.L4);

    state.clear();

    for (var pipe : ReefPipe.values()) {
      for (var level : TRACKED_LEVELS) {
        assertFalse(state.isScored(pipe, level));
      }
    }
  }

  @Test
  public void doubleScoreTest() {
    var state = new ReefState();

    state.markScored(ReefPipe.PIPE_A, ReefPipeLevel.L4);
    assertTrue(state.isScored(ReefPipe.PIPE_A, ReefPipeLevel.L4));

    // Shouldn't change state
    state.markScored(ReefPipe.PIPE_A, ReefPipeLevel.L4);
    assertTrue(state.isScored(ReefPipe.PIPE_A, ReefPipeLevel.L4));
  }
}
