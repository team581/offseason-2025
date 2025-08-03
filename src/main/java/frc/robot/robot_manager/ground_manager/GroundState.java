package frc.robot.robot_manager.ground_manager;

public enum GroundState {
  IDLE_EMPTY,
  IDLE_CORAL,

  INTAKING,

  INTAKE_THEN_HANDOFF_WAIT,

  HANDOFF_WAIT,
  HANDOFF_RELEASE,

  L1_WAIT,
  L1_HARD_WAIT,

  L1_SCORE,
  L1_HARD_SCORE,

  CLIMB,

  REHOME_DEPLOY,
  UNJAM;
}
