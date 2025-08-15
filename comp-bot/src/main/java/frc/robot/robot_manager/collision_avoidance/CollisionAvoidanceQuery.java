package frc.robot.robot_manager.collision_avoidance;

public record CollisionAvoidanceQuery(
    Waypoint currentWaypoint,
    Waypoint goalWaypoint,
    ObstructionKind obstructionKind,
    boolean teleop) {}
