package frc.robot.autos.trackers.pure_pursuit;

import static org.junit.jupiter.api.Assertions.assertTrue;

import com.team581.trailblazer.AutoPoint;
import com.team581.trailblazer.trackers.pure_pursuit.PurePursuitUtils;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import java.util.ArrayList;
import java.util.List;
import org.junit.jupiter.api.Assertions;
import org.junit.jupiter.api.Test;

public class PurePursuitUtilsTest {
  @Test
  void getPerpindicularWaypoint() {
    var startPoint = new Pose2d(-1, -4, Rotation2d.kZero);
    var endPoint = new Pose2d(5, 2, Rotation2d.kZero);
    var robotPose = new Pose2d(-4.0, -4.0, Rotation2d.kZero);
    var result = PurePursuitUtils.getPerpendicularPoint(startPoint, endPoint, robotPose);

    var expected = new Pose2d(-2.5, -5.5, Rotation2d.kZero);
    Assertions.assertEquals(expected, result);
  }

  @Test
  void getLookaheadWaypoint() {
    var startPoint = new Pose2d(0, 0, Rotation2d.kZero);
    var endPoint = new Pose2d(5, 0, Rotation2d.kZero);
    var pointOnPath = new Pose2d(2, 0, Rotation2d.kZero);
    var result = PurePursuitUtils.getLookaheadPoint(startPoint, endPoint, pointOnPath, 1.0);

    var expected = new Pose2d(3, 0, Rotation2d.kZero);
    Assertions.assertEquals(expected, result);
  }

  @Test
  void randomNumber() {
    var min = 0;
    var max = 15;
    var iterations = 1000;

    for (int i = 0; i < iterations; i++) {
      var random = PurePursuitUtils.randomBetween(min, max);
      Assertions.assertTrue(random >= min && random <= max);
    }
  }

  @Test
  void randomPose() {
    var iterations = 1000;
    for (int i = 0; i < iterations; i++) {
      var result = PurePursuitUtils.generateRandomPose();
      Assertions.assertTrue(
          result.getX() >= 0 && result.getX() <= 15 && result.getY() >= 0 && result.getY() <= 8);
    }
  }

  @Test
  void getTargetPoseIsAlwaysInBetweenPoints() {
    var pointsAmount = 10;
    int currentIndex = 3;
    var lookaheadDistance = 1.0;
    var startingRobotPose = PurePursuitUtils.generateRandomPose();
    var iterations = 10000;

    for (int i = 0; i < iterations; i++) {
      List<AutoPoint> points = new ArrayList<>();
      for (int n = 0; n < pointsAmount; n++) {
        points.add(new AutoPoint(PurePursuitUtils.generateRandomPose()));
      }
      var currentRobotPose = PurePursuitUtils.generateRandomPose();
      var targetPose =
          PurePursuitUtils.getTargetPose(
              currentRobotPose, points, currentIndex, lookaheadDistance, startingRobotPose);
      assertTrue(
          PurePursuitUtils.isBetweenAndCollinearWithAnyPoints(
              startingRobotPose, points, targetPose));
    }
  }

  @Test
  void getPerpendicularPointCollinear() {
    var iterations = 100000;
    for (int i = 0; i < iterations; i++) {
      var startPose = PurePursuitUtils.generateRandomPose();
      var endPose = PurePursuitUtils.generateRandomPose();
      var robotPose = PurePursuitUtils.generateRandomPose();
      var result = PurePursuitUtils.getPerpendicularPoint(startPose, endPose, robotPose);
      assertTrue(PurePursuitUtils.isCollinear(startPose, endPose, result));
    }
  }

  @Test
  void pointOnStartPointIsBetween() {
    var iterations = 100000;
    for (int i = 0; i < iterations; i++) {
      var startPose = PurePursuitUtils.generateRandomPose();
      var endPose = PurePursuitUtils.generateRandomPose();
      var between = startPose;
      assertTrue(PurePursuitUtils.isBetween(startPose, endPose, between));
    }
  }

  @Test
  void getPerpendicularLookaheadPointCollinear() {
    var iterations = 100000;
    for (int i = 0; i < iterations; i++) {
      var startPose = PurePursuitUtils.generateRandomPose();
      var endPose = PurePursuitUtils.generateRandomPose();
      var robotPose = PurePursuitUtils.generateRandomPose();
      var perpendicularPoint =
          PurePursuitUtils.getPerpendicularPoint(startPose, endPose, robotPose);
      var result = PurePursuitUtils.getLookaheadPoint(startPose, endPose, perpendicularPoint, 1.0);
      assertTrue(PurePursuitUtils.isCollinear(startPose, endPose, result));
    }
  }

  @Test
  void getInterpolatedRotation() {
    var startPose = PurePursuitUtils.generateRandomPose();
    var endPose = new Pose2d(startPose.getX(), startPose.getY(), startPose.getRotation());
    var pointOnPath = new Pose2d(startPose.getX(), startPose.getY(), startPose.getRotation());
    var result =
        PurePursuitUtils.getPointToPointInterpolatedRotation(startPose, endPose, pointOnPath)
            .getDegrees();
    var expected = endPose.getRotation().getDegrees();
    assertTrue(expected == result);
  }
}
