package frc.robot.robot_manager.collision_avoidance;

import static org.junit.jupiter.api.Assertions.assertEquals;

import frc.robot.arm.ArmState;
import frc.robot.elevator.ElevatorState;
import frc.robot.robot_manager.SuperstructurePosition;
import java.util.List;
import org.junit.jupiter.api.Test;

public class CollisionAvoidanceTest {
  @Test
  public void test() {
    var currentPosition =
        new SuperstructurePosition(
            ElevatorState.CORAL_SCORE_RELEASE_L3, ArmState.CORAL_SCORE_LEFT_RELEASE_L3);
    var desiredPosition =
        new SuperstructurePosition(ElevatorState.CORAL_HANDOFF, ArmState.CORAL_HANDOFF);
    var result =
        CollisionAvoidance.aStar(currentPosition, desiredPosition, ObstructionKind.LEFT_OBSTRUCTED);
    var expected = List.of(Waypoint.L3_LEFT_PLACE, Waypoint.L3_LEFT_ARM, Waypoint.HANDOFF);
    assertEquals(expected, result.get());
  }

  @Test
  public void test1() {
    var currentPosition =
        new SuperstructurePosition(
            ElevatorState.CORAL_SCORE_RELEASE_L3, ArmState.CORAL_SCORE_LEFT_RELEASE_L3);
    var desiredPosition =
        new SuperstructurePosition(ElevatorState.CORAL_HANDOFF, ArmState.CORAL_HANDOFF);
    var result = CollisionAvoidance.aStar(currentPosition, desiredPosition, ObstructionKind.NONE);
    var expected = List.of(Waypoint.L3_LEFT_PLACE, Waypoint.L3_LEFT_ARM, Waypoint.HANDOFF);
    assertEquals(expected, result.get());
  }


  @Test
  public void hectorTest() {
    var currentRawAngle = 720;
    var normalizedGoalAngle = 90;
    var result =
        List.of(
            CollisionAvoidance.getCollisionAvoidanceSolutions(currentRawAngle, normalizedGoalAngle)[
                0],
            CollisionAvoidance.getCollisionAvoidanceSolutions(currentRawAngle, normalizedGoalAngle)[
                1]);
    var expected = List.of(810.0, 450.0);
    assertEquals(expected, result);
  }

  @Test
  public void hectorTest1() {
    var currentRawAngle = -720;
    var normalizedGoalAngle = 90;
    var result =
        List.of(
            CollisionAvoidance.getCollisionAvoidanceSolutions(currentRawAngle, normalizedGoalAngle)[
                0],
            CollisionAvoidance.getCollisionAvoidanceSolutions(currentRawAngle, normalizedGoalAngle)[
                1]);
    var expected = List.of(-630.0, -990.0);
    assertEquals(expected, result);
  }

  @Test
  public void hectorTest4() {
    var normalizedGoalAngle = -90;
    var currentRawAngle = -179;
    var result =
        List.of(
            CollisionAvoidance.getCollisionAvoidanceSolutions(currentRawAngle, normalizedGoalAngle)[
                0],
            CollisionAvoidance.getCollisionAvoidanceSolutions(currentRawAngle, normalizedGoalAngle)[
                1]);
    var expected = List.of(-90.0, -450.0);
    assertEquals(expected, result);
  }

  @Test
  public void hectorTest5() {
    var normalizedGoalAngle = -90;
    var currentRawAngle = 147.7;
    var result =
        List.of(
            CollisionAvoidance.getCollisionAvoidanceSolutions(currentRawAngle, normalizedGoalAngle)[
                0],
            CollisionAvoidance.getCollisionAvoidanceSolutions(currentRawAngle, normalizedGoalAngle)[
                1]);
    var expected = List.of(270.0, -90.0);
    assertEquals(expected, result);
  }

  @Test
  public void hectorTest6() {
    var normalizedGoalAngle = -90;
    var currentRawAngle = -147.7;
    var result =
        List.of(
            CollisionAvoidance.getCollisionAvoidanceSolutions(currentRawAngle, normalizedGoalAngle)[
                0],
            CollisionAvoidance.getCollisionAvoidanceSolutions(currentRawAngle, normalizedGoalAngle)[
                1]);
    var expected = List.of(-90.0, -450.0);
    assertEquals(expected, result);
  }

  @Test
  public void armSetCollisionAvoidanceGoalTest() {
    double goalAngle = -90.0;
    boolean climberRisky = false;
    double currentAngle = -211.0;
    ObstructionKind obstructionKind = ObstructionKind.NONE;
    ObstructionStrategy leftStrategy = ObstructionStrategy.LONG_WAY_IF_BLOCKED;
    ObstructionStrategy rightStrategy = ObstructionStrategy.IGNORE_BLOCKED;

    double result =
        CollisionAvoidance.getCollisionAvoidanceAngleGoal(
            goalAngle, climberRisky, obstructionKind, leftStrategy, rightStrategy, currentAngle);

    double expected = -90.0;

    assertEquals(expected, result);
  }

  @Test
  public void solutionsTest() {
    var normalizedGoalAngle = 160;
    var currentRawAngle = 269.0;
    var result =
        List.of(
            CollisionAvoidance.getCollisionAvoidanceSolutions(currentRawAngle, normalizedGoalAngle)[
                0],
            CollisionAvoidance.getCollisionAvoidanceSolutions(currentRawAngle, normalizedGoalAngle)[
                1]);
    var expected = List.of(160.0, 520.0);
    assertEquals(expected, result);
  }

  @Test
  public void armSetCollisionAvoidanceGoalTestL3Lineup() {
    double goalAngle = 160.0;
    boolean climberRisky = false;
    double currentAngle = 269.0;
    ObstructionKind obstructionKind = ObstructionKind.LEFT_OBSTRUCTED;
    ObstructionStrategy leftStrategy = ObstructionStrategy.LONG_WAY_IF_BLOCKED;
    ObstructionStrategy rightStrategy = ObstructionStrategy.IGNORE_BLOCKED;

    double result =
        CollisionAvoidance.getCollisionAvoidanceAngleGoal(
            goalAngle, climberRisky, obstructionKind, leftStrategy, rightStrategy, currentAngle);

    double expected = 520.0;

    assertEquals(expected, result);
  }

  @Test
  public void positivearmSetCollisionAvoidanceGoalTest() {
    double goalAngle = -90.0;
    boolean climberRisky = false;
    double currentAngle = 140.0;
    ObstructionKind obstructionKind = ObstructionKind.LEFT_OBSTRUCTED;
    ObstructionStrategy leftStrategy = ObstructionStrategy.LONG_WAY_IF_BLOCKED;
    ObstructionStrategy rightStrategy = ObstructionStrategy.IGNORE_BLOCKED;

    double result =
        CollisionAvoidance.getCollisionAvoidanceAngleGoal(
            goalAngle, climberRisky, obstructionKind, leftStrategy, rightStrategy, currentAngle);

    double expected = -90.0;

    assertEquals(expected, result);
  }

  @Test
  public void swingsaroundtwicewhenstowing() { // TODO: make this pass
    double goalAngle = -90.0;
    boolean climberRisky = false;
    double currentAngle = -147.7;
    ObstructionKind obstructionKind = ObstructionKind.LEFT_OBSTRUCTED;
    ObstructionStrategy leftStrategy = ObstructionStrategy.LONG_WAY_IF_BLOCKED;
    ObstructionStrategy rightStrategy = ObstructionStrategy.IGNORE_BLOCKED;

    double result =
        CollisionAvoidance.getCollisionAvoidanceAngleGoal(
            goalAngle, climberRisky, obstructionKind, leftStrategy, rightStrategy, currentAngle);

    double expected = -450.0;

    assertEquals(expected, result);
  }

  @Test
  public void swingsaroundtwicewhenstowin324g() { // TODO: make this pass
    double goalAngle = -90.0;
    boolean climberRisky = false;
    double currentAngle = 147.7;
    ObstructionKind obstructionKind = ObstructionKind.LEFT_OBSTRUCTED;
    ObstructionStrategy leftStrategy = ObstructionStrategy.LONG_WAY_IF_BLOCKED;
    ObstructionStrategy rightStrategy = ObstructionStrategy.IGNORE_BLOCKED;

    double result =
        CollisionAvoidance.getCollisionAvoidanceAngleGoal(
            goalAngle, climberRisky, obstructionKind, leftStrategy, rightStrategy, currentAngle);

    double expected = -90.0;

    assertEquals(expected, result);
  }

  @Test
  public void armSetCollisionAvoidanceGoalTest3() {
    double goalAngle = 180.0;
    boolean climberRisky = false;
    double currentAngle = -90.0;
    ObstructionKind obstructionKind = ObstructionKind.RIGHT_OBSTRUCTED;
    ObstructionKind edgeObstructionKind = ObstructionKind.RIGHT_OBSTRUCTED;
    ObstructionStrategy leftStrategy = ObstructionStrategy.LONG_WAY_IF_BLOCKED;
    ObstructionStrategy rightStrategy = ObstructionStrategy.LONG_WAY_IF_BLOCKED;
    double result =
        CollisionAvoidance.getCollisionAvoidanceAngleGoal(
            goalAngle, climberRisky, obstructionKind, leftStrategy, rightStrategy, currentAngle);

    double expected = 180.0;

    assertEquals(expected, result);
  }

  @Test
  public void armSetCollisionAvoidanceGoalBackwardTest() {
    double goalAngle = -25.0;
    boolean climberRisky = true;
    double currentAngle = 0.0;
    ObstructionKind obstructionKind = ObstructionKind.LEFT_OBSTRUCTED;
    ObstructionKind edgeObstructionKind = ObstructionKind.LEFT_OBSTRUCTED;
    ObstructionStrategy leftStrategy = ObstructionStrategy.LONG_WAY_IF_BLOCKED;
    ObstructionStrategy rightStrategy = ObstructionStrategy.LONG_WAY_IF_BLOCKED;
    double result =
        CollisionAvoidance.getCollisionAvoidanceAngleGoal(
            goalAngle, climberRisky, obstructionKind, leftStrategy, rightStrategy, currentAngle);

    double expected = -25.0;

    assertEquals(expected, result);
  }

  @Test
  public void armSetCollisionAvoidanceGoalGoLongWayTest() {
    double goalAngle = 200.0;
    boolean climberRisky = true;
    double currentAngle = 360.0;
    ObstructionKind obstructionKind = ObstructionKind.LEFT_OBSTRUCTED;
    ObstructionKind edgeObstructionKind = ObstructionKind.LEFT_OBSTRUCTED;
    ObstructionStrategy leftStrategy = ObstructionStrategy.LONG_WAY_IF_BLOCKED;
    ObstructionStrategy rightStrategy = ObstructionStrategy.LONG_WAY_IF_BLOCKED;
    double result =
        CollisionAvoidance.getCollisionAvoidanceAngleGoal(
            goalAngle, climberRisky, obstructionKind, leftStrategy, rightStrategy, currentAngle);

    double expected = 560.0;

    assertEquals(expected, result);
  }

  @Test
  public void armSetCollisionAvoidanceGoalGoLongWayNegativeTest() {
    double goalAngle = -200.0;
    boolean climberRisky = true;
    double currentAngle = -360.0;
    ObstructionKind obstructionKind = ObstructionKind.LEFT_OBSTRUCTED;
    ObstructionKind edgeObstructionKind = ObstructionKind.LEFT_OBSTRUCTED;
    ObstructionStrategy leftStrategy = ObstructionStrategy.LONG_WAY_IF_BLOCKED;
    ObstructionStrategy rightStrategy = ObstructionStrategy.LONG_WAY_IF_BLOCKED;
    double result =
        CollisionAvoidance.getCollisionAvoidanceAngleGoal(
            goalAngle, climberRisky, obstructionKind, leftStrategy, rightStrategy, currentAngle);

    double expected = -200.0;

    assertEquals(expected, result);
  }

  @Test
  public void armSetCollisionAvoidanceGoalNegativeTest() {
    double goalAngle = 180.0;
    boolean climberRisky = true;
    double currentAngle = -360.0;
    ObstructionKind obstructionKind = ObstructionKind.LEFT_OBSTRUCTED;
    ObstructionKind edgeObstructionKind = ObstructionKind.LEFT_OBSTRUCTED;
    ObstructionStrategy leftStrategy = ObstructionStrategy.LONG_WAY_IF_BLOCKED;
    ObstructionStrategy rightStrategy = ObstructionStrategy.LONG_WAY_IF_BLOCKED;
    double result =
        CollisionAvoidance.getCollisionAvoidanceAngleGoal(
            goalAngle, climberRisky, obstructionKind, leftStrategy, rightStrategy, currentAngle);

    double expected = -180.0;

    assertEquals(expected, result);
  }

  @Test
  public void armSetCollisionAvoidanceGoalPositiveTest() {
    double goalAngle = 180.0;
    boolean climberRisky = true;
    double currentAngle = 360.0;
    ObstructionKind obstructionKind = ObstructionKind.LEFT_OBSTRUCTED;
    ObstructionKind edgeObstructionKind = ObstructionKind.LEFT_OBSTRUCTED;
    ObstructionStrategy leftStrategy = ObstructionStrategy.LONG_WAY_IF_BLOCKED;
    ObstructionStrategy rightStrategy = ObstructionStrategy.LONG_WAY_IF_BLOCKED;
    double result =
        CollisionAvoidance.getCollisionAvoidanceAngleGoal(
            goalAngle, climberRisky, obstructionKind, leftStrategy, rightStrategy, currentAngle);

    double expected = 540.0;

    assertEquals(expected, result);
  }

  // @Test
  // public void leftObstructedAstarTest() {
  //   var result =
  //       CollisionAvoidance.aStar(
  //           new SuperstructurePosition(0, 90),
  //           new SuperstructurePosition(50, 180),
  //           ObstructionKind.LEFT_OBSTRUCTED);
  //   var expected =
  //       List.of(
  //           Waypoint.ELEVATOR_0_ARM_UP,
  //           Waypoint.CLIMBER_SAFE_ARM_UP,
  //           Waypoint.L4_LEFT,
  //           Waypoint.L4_LEFT_PLACE);
  //   assertEquals(expected, result.get());
  // }

  @Test
  public void rightObstructedAstarTest() {
    var result =
        CollisionAvoidance.aStar(
            new SuperstructurePosition(ElevatorState.STOWED, ArmState.HOLDING_UPRIGHT),
            new SuperstructurePosition(
                ElevatorState.CORAL_SCORE_LINEUP_L4, ArmState.CORAL_SCORE_RIGHT_LINEUP_L4),
            ObstructionKind.RIGHT_OBSTRUCTED);
    // Moves elevator first, only extends arm at the end
    var expected = List.of(Waypoint.L1_UPRIGHT, Waypoint.L4_UPRIGHT, Waypoint.L4_RIGHT_LINEUP);

    assertEquals(expected, result.get());
  }

  @Test
  public void getClosestNodeStowedTest() {
    var result = Waypoint.getClosest(new SuperstructurePosition(43, -90));
    Waypoint expected = Waypoint.HANDOFF;

    assertEquals(expected, result);
  }

  @Test
  void stowsSafelyAfterRightNetScoreTest() {
    var result =
        CollisionAvoidance.aStar(
            new SuperstructurePosition(ElevatorState.ALGAE_NET, ArmState.ALGAE_NET_RIGHT),
            new SuperstructurePosition(ElevatorState.PRE_CORAL_HANDOFF, ArmState.CORAL_HANDOFF),
            ObstructionKind.NONE);
    var expected =
        List.of(
            Waypoint.ALGAE_NET_OUT_RIGHT,
            Waypoint.ALGAE_NET_UP,
            Waypoint.HANDOFF_CLEARS_CLIMBER,
            Waypoint.HANDOFF);

    assertEquals(expected, result.orElseThrow());
  }


  @Test
  void handoffToLollipop() {
    var result =
        CollisionAvoidance.aStar(
            new SuperstructurePosition(ElevatorState.PRE_CORAL_HANDOFF, ArmState.CORAL_HANDOFF),
            new SuperstructurePosition(
                ElevatorState.LOLLIPOP_CORAL_INTAKE_INTAKE, ArmState.LOLLIPOP_CORAL_INTAKE_INTAKE),
            ObstructionKind.NONE);
    var expected = List.of(Waypoint.HANDOFF, Waypoint.L2_RIGHT_ARM, Waypoint.LOLLIPOP_INTAKE_RIGHT);

    assertEquals(expected, result.orElseThrow());
  }

  @Test
  void l4PlaceToLollipop() {
    var result =
        CollisionAvoidance.aStar(
            new SuperstructurePosition(
                ElevatorState.CORAL_SCORE_RELEASE_L4, ArmState.CORAL_SCORE_LEFT_RELEASE_L4),
            new SuperstructurePosition(
                ElevatorState.LOLLIPOP_CORAL_INTAKE_PUSH, ArmState.LOLLIPOP_CORAL_INTAKE_PUSH),
            ObstructionKind.LEFT_OBSTRUCTED);
    var expected = List.of(Waypoint.L4_LEFT_PLACE, Waypoint.LOLLIPOP_INTAKE_RIGHT);

    assertEquals(expected, result.orElseThrow());
  }

  @Test
  void lollipopPushToL4LineupTest() {
    var result =
        CollisionAvoidance.aStar(
            Waypoint.LOLLIPOP_INTAKE_PUSH,
            new SuperstructurePosition(
                ElevatorState.CORAL_SCORE_LINEUP_L4, ArmState.CORAL_SCORE_LEFT_LINEUP_L4),
            ObstructionKind.NONE);
    var expected = List.of(Waypoint.LOLLIPOP_INTAKE_PUSH, Waypoint.L4_LEFT_LINEUP);

    assertEquals(expected, result.orElseThrow());
  }

  @Test
  void l2PlaceToLollipop() {
    var result =
        CollisionAvoidance.aStar(
            new SuperstructurePosition(
                ElevatorState.CORAL_SCORE_RELEASE_L2, ArmState.CORAL_SCORE_LEFT_RELEASE_L2),
            new SuperstructurePosition(
                ElevatorState.LOLLIPOP_CORAL_INTAKE_PUSH, ArmState.LOLLIPOP_CORAL_INTAKE_PUSH),
            ObstructionKind.LEFT_OBSTRUCTED);
    var expected =
        List.of(
            Waypoint.L2_LEFT_PLACE,
            Waypoint.L2_LEFT_LINEUP,
            Waypoint.LOLLIPOP_INTAKE_PUSH,
            Waypoint.LOLLIPOP_INTAKE_RIGHT);

    assertEquals(expected, result.orElseThrow());
  }

  @Test
  void lollipopPushToL2LineupTest() {
    var result =
        CollisionAvoidance.aStar(
            Waypoint.LOLLIPOP_INTAKE_PUSH,
            new SuperstructurePosition(
                ElevatorState.CORAL_SCORE_LINEUP_L2, ArmState.CORAL_SCORE_LEFT_LINEUP_L2),
            ObstructionKind.NONE);
    var expected = List.of(Waypoint.LOLLIPOP_INTAKE_PUSH, Waypoint.L2_LEFT_LINEUP);

    assertEquals(expected, result.orElseThrow());
  }

  @Test
  void l3PlaceToLollipop() {
    var result =
        CollisionAvoidance.aStar(
            new SuperstructurePosition(
                ElevatorState.CORAL_SCORE_RELEASE_L3, ArmState.CORAL_SCORE_LEFT_RELEASE_L3),
            new SuperstructurePosition(
                ElevatorState.LOLLIPOP_CORAL_INTAKE_PUSH, ArmState.LOLLIPOP_CORAL_INTAKE_PUSH),
            ObstructionKind.LEFT_OBSTRUCTED);
    var expected = List.of(Waypoint.L3_LEFT_PLACE, Waypoint.LOLLIPOP_INTAKE_RIGHT);

    assertEquals(expected, result.orElseThrow());
  }

  @Test
  void lollipopPushToL3LineupTest() {
    var result =
        CollisionAvoidance.aStar(
            Waypoint.LOLLIPOP_INTAKE_PUSH,
            new SuperstructurePosition(
                ElevatorState.CORAL_SCORE_LINEUP_L3, ArmState.CORAL_SCORE_LEFT_LINEUP_L3),
            ObstructionKind.NONE);
    var expected = List.of(Waypoint.LOLLIPOP_INTAKE_PUSH, Waypoint.L3_LEFT_LINEUP);

    assertEquals(expected, result.orElseThrow());
  }

  @Test
  void leftObstructedL2toL3Test() {
    var result =
        CollisionAvoidance.aStar(
            new SuperstructurePosition(
                ElevatorState.CORAL_SCORE_LINEUP_L2, ArmState.CORAL_SCORE_LEFT_LINEUP_L2),
            new SuperstructurePosition(
                ElevatorState.CORAL_SCORE_LINEUP_L3, ArmState.CORAL_SCORE_LEFT_LINEUP_L3),
            ObstructionKind.LEFT_OBSTRUCTED);
    var expected =
        List.of(
            Waypoint.L2_LEFT_LINEUP,
            Waypoint.L2_UPRIGHT,
            Waypoint.L3_UPRIGHT,
            Waypoint.L3_LEFT_LINEUP);

    assertEquals(expected, result.orElseThrow());
  }

  @Test
  void leftObstructedL2PlaceTest() {
    var result =
        CollisionAvoidance.aStar(
            new SuperstructurePosition(
                ElevatorState.CORAL_SCORE_LINEUP_L2, ArmState.CORAL_SCORE_LEFT_LINEUP_L2),
            new SuperstructurePosition(
                ElevatorState.CORAL_SCORE_RELEASE_L2, ArmState.CORAL_SCORE_LEFT_RELEASE_L2),
            ObstructionKind.LEFT_OBSTRUCTED);

    var expected = List.of(Waypoint.L2_LEFT_LINEUP, Waypoint.L2_LEFT_PLACE);

    assertEquals(expected, result.orElseThrow());
  }

  @Test
  void leftObstructedL2UprightToLineup() {
    var result =
        CollisionAvoidance.aStar(
            new SuperstructurePosition(
                ElevatorState.CORAL_SCORE_LINEUP_L2, ArmState.HOLDING_UPRIGHT),
            new SuperstructurePosition(
                ElevatorState.CORAL_SCORE_LINEUP_L2, ArmState.CORAL_SCORE_LEFT_LINEUP_L2),
            ObstructionKind.LEFT_OBSTRUCTED);

    var expected = List.of(Waypoint.L2_UPRIGHT, Waypoint.L2_LEFT_LINEUP);

    assertEquals(expected, result.orElseThrow());
  }

  @Test
  void notObstructedL2UprightToLineup() {
    var result =
        CollisionAvoidance.aStar(
            new SuperstructurePosition(
                ElevatorState.CORAL_SCORE_LINEUP_L2, ArmState.HOLDING_UPRIGHT),
            new SuperstructurePosition(
                ElevatorState.CORAL_SCORE_LINEUP_L2, ArmState.CORAL_SCORE_LEFT_LINEUP_L2),
            ObstructionKind.NONE);

    var expected = List.of(Waypoint.L2_UPRIGHT, Waypoint.L2_LEFT_LINEUP);

    assertEquals(expected, result.orElseThrow());
  }

  @Test
  void leftObstructedL4LeftToL2LeftTest() {
    var result =
        CollisionAvoidance.aStar(
            new SuperstructurePosition(
                ElevatorState.CORAL_SCORE_LINEUP_L4, ArmState.CORAL_SCORE_LEFT_LINEUP_L4),
            new SuperstructurePosition(
                ElevatorState.CORAL_SCORE_LINEUP_L2, ArmState.CORAL_SCORE_LEFT_LINEUP_L2),
            ObstructionKind.LEFT_OBSTRUCTED);
    var expected =
        List.of(
            Waypoint.L4_LEFT_LINEUP,
            Waypoint.L4_UPRIGHT,
            Waypoint.L2_UPRIGHT,
            Waypoint.L2_LEFT_LINEUP);

    assertEquals(expected, result.orElseThrow());
  }

 
}
