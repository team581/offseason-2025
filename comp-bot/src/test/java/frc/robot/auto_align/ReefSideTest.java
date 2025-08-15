package frc.robot.auto_align;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.util.MathHelpers;
import org.junit.jupiter.api.Test;

public class ReefSideTest {
  private static void assertSidePoseCorect(boolean isRed, Pose2d robotPose, ReefSide side) {
    var pipe1Distance =
        side.pipe1
            .getPose(ReefPipeLevel.BASE, isRed, RobotScoringSide.RIGHT)
            .getTranslation()
            .getDistance(side.getPose(robotPose).getTranslation());
    var pipe2Distance =
        side.pipe2
            .getPose(ReefPipeLevel.BASE, isRed, RobotScoringSide.RIGHT)
            .getTranslation()
            .getDistance(side.getPose(robotPose).getTranslation());

    assertEquals(MathHelpers.roundTo(pipe1Distance, 2), MathHelpers.roundTo(pipe2Distance, 2));
  }

  private static Pair<ReefSide, ReefSide> getNeighbors(ReefSide side) {
    return switch (side) {
      case SIDE_CD -> new Pair<>(ReefSide.SIDE_AB, ReefSide.SIDE_EF);
      case SIDE_EF -> new Pair<>(ReefSide.SIDE_CD, ReefSide.SIDE_GH);
      case SIDE_GH -> new Pair<>(ReefSide.SIDE_EF, ReefSide.SIDE_IJ);
      case SIDE_IJ -> new Pair<>(ReefSide.SIDE_GH, ReefSide.SIDE_KL);
      case SIDE_KL -> new Pair<>(ReefSide.SIDE_IJ, ReefSide.SIDE_AB);
      case SIDE_AB -> new Pair<>(ReefSide.SIDE_KL, ReefSide.SIDE_CD);
    };
  }

  private static void assertSideSpacingCorrect(Pose2d robotPose, ReefSide side) {
    var neighbors = getNeighbors(side);

    var sideTranslation = side.getPose(robotPose).getTranslation();

    var distanceToPrevious =
        sideTranslation.getDistance(neighbors.getFirst().getPose(robotPose).getTranslation());
    var distanceToNext =
        sideTranslation.getDistance(neighbors.getSecond().getPose(robotPose).getTranslation());

    assertEquals(
        MathHelpers.roundTo(distanceToPrevious, 2), MathHelpers.roundTo(distanceToNext, 2));
  }

  @Test
  void sideAbRedTest() {
    assertSidePoseCorect(true, new Pose2d(15, 0, Rotation2d.kZero), ReefSide.SIDE_AB);
    assertSideSpacingCorrect(new Pose2d(15, 0, Rotation2d.kZero), ReefSide.SIDE_AB);
  }

  @Test
  void sideAbBlueTest() {
    assertSidePoseCorect(false, new Pose2d(1, 0, Rotation2d.kZero), ReefSide.SIDE_AB);
    assertSideSpacingCorrect(new Pose2d(1, 0, Rotation2d.kZero), ReefSide.SIDE_AB);
  }

  @Test
  void sideCdRedTest() {
    assertSidePoseCorect(true, new Pose2d(15, 0, Rotation2d.kZero), ReefSide.SIDE_CD);
    assertSideSpacingCorrect(new Pose2d(15, 0, Rotation2d.kZero), ReefSide.SIDE_CD);
  }

  @Test
  void sideCdBlueTest() {
    assertSidePoseCorect(false, new Pose2d(1, 0, Rotation2d.kZero), ReefSide.SIDE_CD);
    assertSideSpacingCorrect(new Pose2d(1, 0, Rotation2d.kZero), ReefSide.SIDE_CD);
  }

  @Test
  void sideEfRedTest() {
    assertSidePoseCorect(true, new Pose2d(15, 0, Rotation2d.kZero), ReefSide.SIDE_EF);
    assertSideSpacingCorrect(new Pose2d(15, 0, Rotation2d.kZero), ReefSide.SIDE_EF);
  }

  @Test
  void sideEfBlueTest() {
    assertSidePoseCorect(false, new Pose2d(1, 0, Rotation2d.kZero), ReefSide.SIDE_EF);
    assertSideSpacingCorrect(new Pose2d(1, 0, Rotation2d.kZero), ReefSide.SIDE_EF);
  }

  @Test
  void sideGhRedTest() {
    assertSidePoseCorect(true, new Pose2d(15, 0, Rotation2d.kZero), ReefSide.SIDE_GH);
    assertSideSpacingCorrect(new Pose2d(15, 0, Rotation2d.kZero), ReefSide.SIDE_GH);
  }

  @Test
  void sideGhBlueTest() {
    assertSidePoseCorect(false, new Pose2d(1, 0, Rotation2d.kZero), ReefSide.SIDE_GH);
    assertSideSpacingCorrect(new Pose2d(1, 0, Rotation2d.kZero), ReefSide.SIDE_GH);
  }

  @Test
  void sideIjRedTest() {
    assertSidePoseCorect(true, new Pose2d(15, 0, Rotation2d.kZero), ReefSide.SIDE_IJ);
    assertSideSpacingCorrect(new Pose2d(15, 0, Rotation2d.kZero), ReefSide.SIDE_IJ);
  }

  @Test
  void sideIjBlueTest() {
    assertSidePoseCorect(false, new Pose2d(1, 0, Rotation2d.kZero), ReefSide.SIDE_IJ);
    assertSideSpacingCorrect(new Pose2d(1, 0, Rotation2d.kZero), ReefSide.SIDE_IJ);
  }

  @Test
  void sideKlRedTest() {
    assertSidePoseCorect(true, new Pose2d(15, 0, Rotation2d.kZero), ReefSide.SIDE_KL);
    assertSideSpacingCorrect(new Pose2d(15, 0, Rotation2d.kZero), ReefSide.SIDE_KL);
  }

  @Test
  void sideKlBlueTest() {
    assertSidePoseCorect(false, new Pose2d(1, 0, Rotation2d.kZero), ReefSide.SIDE_KL);
    assertSideSpacingCorrect(new Pose2d(1, 0, Rotation2d.kZero), ReefSide.SIDE_KL);
  }
}
