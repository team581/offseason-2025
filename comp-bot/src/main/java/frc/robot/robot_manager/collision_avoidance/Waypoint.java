package frc.robot.robot_manager.collision_avoidance;

import com.google.common.graph.MutableValueGraph;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Translation2d;
import frc.robot.arm.ArmState;
import frc.robot.config.FeatureFlags;
import frc.robot.elevator.ElevatorState;
import frc.robot.robot_manager.SuperstructurePosition;
import java.util.Comparator;
import java.util.List;
import java.util.stream.Stream;

/**
 * These represent "waypoints" for collision avoidance to route through. These are NOT setpoints
 * that the robot uses, even though they may share a name and/or superstructure position. Collision
 * avoidance uses these as nodes within a graph to route from a current position to a goal position.
 */
public enum Waypoint {
  ELEVATOR_HANDOFF_ARM_GROUND_ALGAE_INTAKE(
      new SuperstructurePosition(
          ElevatorState.PRE_CORAL_HANDOFF.getHeight() + 15.0, ArmState.ALGAE_INTAKE_FLOOR)),
  GROUND_ALGAE_INTAKE(
      new SuperstructurePosition(ElevatorState.ALGAE_INTAKE_GROUND, ArmState.ALGAE_INTAKE_FLOOR)),
  LOLLIPOP_INTAKE_RIGHT(
      new SuperstructurePosition(
          ElevatorState.LOLLIPOP_CORAL_INTAKE_INTAKE, ArmState.LOLLIPOP_CORAL_INTAKE_INTAKE)),
  LOLLIPOP_INTAKE_PUSH(
      new SuperstructurePosition(
          ElevatorState.LOLLIPOP_CORAL_INTAKE_PUSH, ArmState.LOLLIPOP_CORAL_INTAKE_PUSH)),
  HANDOFF(new SuperstructurePosition(ElevatorState.PRE_CORAL_HANDOFF, ArmState.CORAL_HANDOFF)),
  HANDOFF_CLEARS_CLIMBER(new SuperstructurePosition(55, ArmState.CORAL_HANDOFF)),

  PROCESSOR(new SuperstructurePosition(ElevatorState.PROCESSOR, ArmState.ALGAE_PROCESSOR)),
  CLIMB(new SuperstructurePosition(ElevatorState.CLIMBING, ArmState.CLIMBING)),

  L1_UPRIGHT(new SuperstructurePosition(ElevatorState.STOWED, ArmState.HOLDING_UPRIGHT)),
  L2_UPRIGHT(
      new SuperstructurePosition(ElevatorState.CORAL_SCORE_LINEUP_L2, ArmState.HOLDING_UPRIGHT)),
  L3_UPRIGHT(
      new SuperstructurePosition(ElevatorState.CORAL_SCORE_LINEUP_L3, ArmState.HOLDING_UPRIGHT)),
  L4_UPRIGHT(
      new SuperstructurePosition(ElevatorState.CORAL_SCORE_LINEUP_L4, ArmState.HOLDING_UPRIGHT)),
  L2_RIGHT_ARM(
      new SuperstructurePosition(
          ElevatorState.PRE_CORAL_HANDOFF, ArmState.CORAL_SCORE_RIGHT_LINEUP_L2.getAngle() - 60)),
  L2_LEFT_ARM(
      new SuperstructurePosition(
          ElevatorState.PRE_CORAL_HANDOFF, ArmState.CORAL_SCORE_LEFT_LINEUP_L2.getAngle() + 60)),
  L3_RIGHT_ARM(
      new SuperstructurePosition(
          ElevatorState.PRE_CORAL_HANDOFF, ArmState.CORAL_SCORE_RIGHT_LINEUP_L3.getAngle() - 60)),
  L3_LEFT_ARM(
      new SuperstructurePosition(
          ElevatorState.PRE_CORAL_HANDOFF, ArmState.CORAL_SCORE_LEFT_LINEUP_L3.getAngle() + 60)),

  L2_RIGHT_ARM_ALGAE(
      new SuperstructurePosition(
          ElevatorState.PRE_CORAL_HANDOFF, ArmState.CORAL_SCORE_RIGHT_LINEUP_L2.getAngle() - 60)),
  L2_LEFT_ARM_ALGAE(
      new SuperstructurePosition(
          ElevatorState.PRE_CORAL_HANDOFF, ArmState.CORAL_SCORE_LEFT_LINEUP_L2.getAngle() + 60)),
  L3_RIGHT_ARM_ALGAE(
      new SuperstructurePosition(
          ElevatorState.PRE_CORAL_HANDOFF, ArmState.CORAL_SCORE_RIGHT_LINEUP_L3.getAngle() - 60)),
  L3_LEFT_ARM_ALGAE(
      new SuperstructurePosition(
          ElevatorState.PRE_CORAL_HANDOFF, ArmState.CORAL_SCORE_LEFT_LINEUP_L3.getAngle() + 60)),

  L1_RIGHT_LINEUP(
      new SuperstructurePosition(
          ElevatorState.CORAL_SCORE_RIGHT_LINEUP_L1, ArmState.CORAL_SCORE_RIGHT_LINEUP_L1)),
  L2_RIGHT_LINEUP(
      new SuperstructurePosition(
          ElevatorState.CORAL_SCORE_LINEUP_L2, ArmState.CORAL_SCORE_RIGHT_LINEUP_L2)),
  L2_RIGHT_PLACE(
      new SuperstructurePosition(
          ElevatorState.CORAL_SCORE_RELEASE_L2, ArmState.CORAL_SCORE_RIGHT_RELEASE_L2)),
  L3_RIGHT_LINEUP(
      new SuperstructurePosition(
          ElevatorState.CORAL_SCORE_LINEUP_L3, ArmState.CORAL_SCORE_RIGHT_LINEUP_L3)),
  L3_RIGHT_PLACE(
      new SuperstructurePosition(
          ElevatorState.CORAL_SCORE_RELEASE_L3, ArmState.CORAL_SCORE_RIGHT_RELEASE_L3)),
  L4_RIGHT_LINEUP(
      new SuperstructurePosition(
          ElevatorState.CORAL_SCORE_LINEUP_L4, ArmState.CORAL_SCORE_RIGHT_LINEUP_L4)),
  L4_RIGHT_PLACE(
      new SuperstructurePosition(
          ElevatorState.CORAL_SCORE_RELEASE_L4, ArmState.CORAL_SCORE_RIGHT_RELEASE_L4)),
  L2_LEFT_LINEUP(
      new SuperstructurePosition(
          ElevatorState.CORAL_SCORE_LINEUP_L2, ArmState.CORAL_SCORE_LEFT_LINEUP_L2)),
  L2_LEFT_PLACE(
      new SuperstructurePosition(
          ElevatorState.CORAL_SCORE_RELEASE_L2, ArmState.CORAL_SCORE_LEFT_RELEASE_L2)),
  L3_LEFT_LINEUP(
      new SuperstructurePosition(
          ElevatorState.CORAL_SCORE_LINEUP_L3, ArmState.CORAL_SCORE_LEFT_LINEUP_L3)),
  L3_LEFT_PLACE(
      new SuperstructurePosition(
          ElevatorState.CORAL_SCORE_RELEASE_L3, ArmState.CORAL_SCORE_LEFT_RELEASE_L3)),
  L4_LEFT_LINEUP(
      new SuperstructurePosition(
          ElevatorState.CORAL_SCORE_LINEUP_L4, ArmState.CORAL_SCORE_LEFT_LINEUP_L4)),
  L4_LEFT_PLACE(
      new SuperstructurePosition(
          ElevatorState.CORAL_SCORE_RELEASE_L4, ArmState.CORAL_SCORE_LEFT_RELEASE_L4)),
  ALGAE_NET_UP(new SuperstructurePosition(48.0, ArmState.HOLDING_UPRIGHT)),
  ALGAE_NET_OUT_RIGHT(
      new SuperstructurePosition(ElevatorState.ALGAE_NET, ArmState.ALGAE_NET_RIGHT)),
  ALGAE_NET_OUT_LEFT(new SuperstructurePosition(ElevatorState.ALGAE_NET, ArmState.ALGAE_NET_LEFT)),
  REEF_ALGAE_L2_RIGHT(
      new SuperstructurePosition(ElevatorState.ALGAE_INTAKE_L2, ArmState.ALGAE_INTAKE_RIGHT_L2)),
  REEF_ALGAE_L2_LEFT(
      new SuperstructurePosition(ElevatorState.ALGAE_INTAKE_L2, ArmState.ALGAE_INTAKE_LEFT_L2)),

  REEF_ALGAE_L3_RIGHT(
      new SuperstructurePosition(ElevatorState.ALGAE_INTAKE_L3, ArmState.ALGAE_INTAKE_RIGHT_L3)),
  REEF_ALGAE_L3_LEFT(
      new SuperstructurePosition(ElevatorState.ALGAE_INTAKE_L3, ArmState.ALGAE_INTAKE_LEFT_L3));

  private static final List<Waypoint> ALL_WAYPOINTS = List.of(values());

  public final SuperstructurePosition position;

  Waypoint(SuperstructurePosition position) {
    this.position = position;
  }

  public double costFor(Waypoint other) {
    return position.costFor(other.position);
  }

  public double costForLongWay(Waypoint other) {
    return position.costForLongWay(other.position);
  }

  public static void log() {
    for (var waypoint : values()) {
      DogLog.log(
          "CollisionAvoidance/Waypoints/" + waypoint.toString(), waypoint.position.translation());
    }
    DogLog.log(
        "CollisionAvoidance/Waypoints/All",
        Stream.of(values())
            .map(waypoint -> waypoint.position.translation())
            .toArray(Translation2d[]::new));
  }

  /**
   * Find the closest waypoint to the given superstructure position.
   *
   * @param position The position of the superstructure.
   */
  public static Waypoint getClosest(SuperstructurePosition position) {
    if (FeatureFlags.USE_ALTERNATE_WAYPOINT_CHOOSER.getAsBoolean()) {
      var positionTranslation = position.translation();

      return ALL_WAYPOINTS.stream()
          .min(
              Comparator.comparingDouble(
                  waypoint -> positionTranslation.getDistance(waypoint.position.translation())))
          .orElseThrow();
    }

    return ALL_WAYPOINTS.stream()
        .min(Comparator.comparingDouble(waypoint -> position.costFor(waypoint.position)))
        .orElseThrow();
  }

  public void alwaysSafe(MutableValueGraph<Waypoint, WaypointEdge> graph, Waypoint... others) {
    for (var other : others) {
      graph.putEdgeValue(
          this,
          other,
          new WaypointEdge(
              this, other, ObstructionStrategy.IGNORE_BLOCKED, ObstructionStrategy.IGNORE_BLOCKED));
    }
  }

  public void leftSideSpecial(
      MutableValueGraph<Waypoint, WaypointEdge> graph,
      ObstructionStrategy leftStrategy,
      Waypoint... others) {
    for (var other : others) {
      graph.putEdgeValue(
          this,
          other,
          new WaypointEdge(this, other, leftStrategy, ObstructionStrategy.IGNORE_BLOCKED));
    }
  }

  public void rightSideSpecial(
      MutableValueGraph<Waypoint, WaypointEdge> graph,
      ObstructionStrategy rightStrategy,
      Waypoint... others) {
    for (var other : others) {
      graph.putEdgeValue(
          this,
          other,
          new WaypointEdge(this, other, ObstructionStrategy.IGNORE_BLOCKED, rightStrategy));
    }
  }

  public void avoidClimberAlwaysSafe(
      MutableValueGraph<Waypoint, WaypointEdge> graph, Waypoint... others) {
    for (var other : others) {
      graph.putEdgeValue(
          this,
          other,
          new WaypointEdge(
                  this,
                  other,
                  ObstructionStrategy.IGNORE_BLOCKED,
                  ObstructionStrategy.IGNORE_BLOCKED)
              .avoidClimber());
    }
  }

  public void avoidClimberAlwaysSafeTeleop(
      MutableValueGraph<Waypoint, WaypointEdge> graph, Waypoint... others) {
    for (var other : others) {
      graph.putEdgeValue(
          this,
          other,
          new WaypointEdge(
                  this,
                  other,
                  ObstructionStrategy.IGNORE_BLOCKED,
                  ObstructionStrategy.IGNORE_BLOCKED)
              .avoidClimber()
              .onlyTeleop());
    }
  }

  public void avoidClimberLeftSideSpecial(
      MutableValueGraph<Waypoint, WaypointEdge> graph,
      ObstructionStrategy leftStrategy,
      Waypoint... others) {
    for (var other : others) {
      graph.putEdgeValue(
          this,
          other,
          new WaypointEdge(this, other, leftStrategy, ObstructionStrategy.IGNORE_BLOCKED)
              .avoidClimber());
    }
  }

  public void avoidClimberRightSideSpecial(
      MutableValueGraph<Waypoint, WaypointEdge> graph,
      ObstructionStrategy rightStrategy,
      Waypoint... others) {
    for (var other : others) {
      graph.putEdgeValue(
          this,
          other,
          new WaypointEdge(this, other, ObstructionStrategy.IGNORE_BLOCKED, rightStrategy)
              .avoidClimber());
    }
  }
}
