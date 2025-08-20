package frc.robot.robot_manager.collision_avoidance;

import edu.wpi.first.wpilibj.DriverStation;

public record WaypointEdge(
    double shortCost,
    double longCost,
    boolean hitsClimber,
    /** Whether this motion is safe when the left side of the robot is obstructed. */
    ObstructionStrategy leftSideStrategy,
    /** Whether this motion is safe when the right side of the robot is obstructed. */
    ObstructionStrategy rightSideStrategy,
    boolean teleopOnly) {
  public WaypointEdge(
      Waypoint from,
      Waypoint to,
      ObstructionStrategy leftSideStrategy,
      ObstructionStrategy rightSideStrategy) {
    this(
        from.costFor(to),
        from.costForLongWay(to),
        false,
        leftSideStrategy,
        rightSideStrategy,
        false);
  }

  public WaypointEdge avoidClimber() {
    return new WaypointEdge(
        shortCost, longCost, true, leftSideStrategy, rightSideStrategy, teleopOnly);
  }

  public WaypointEdge onlyTeleop() {
    return new WaypointEdge(
        shortCost, longCost, hitsClimber, leftSideStrategy, rightSideStrategy, true);
  }

  public double getCost(ObstructionKind obstruction) {
    if (teleopOnly && DriverStation.isAutonomous()) {
      return Double.MAX_VALUE;
    }

    return switch (obstruction) {
      case LEFT_OBSTRUCTED ->
          switch (leftSideStrategy) {
            case IMPOSSIBLE_IF_BLOCKED -> Double.MAX_VALUE;
            case IGNORE_BLOCKED -> shortCost;
            case LONG_WAY_IF_BLOCKED -> longCost;
          };
      case RIGHT_OBSTRUCTED ->
          switch (rightSideStrategy) {
            case IMPOSSIBLE_IF_BLOCKED -> Double.MAX_VALUE;
            case IGNORE_BLOCKED -> shortCost;
            case LONG_WAY_IF_BLOCKED -> longCost;
          };
      case NONE -> shortCost;
    };
  }
}
