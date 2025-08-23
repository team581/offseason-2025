package frc.robot.elevator;

public enum ElevatorState {
  UNTUNED(0.0),
  UNJAM(UNTUNED),
  REHOME(UNTUNED),

  STOWED(UNTUNED),
  HANDOFF(UNTUNED),

  L1_SCORE_LINEUP(UNTUNED),
  L1_SCORE_RELEASE(UNTUNED),

  L2_SCORE_LINEUP(UNTUNED),
  L2_SCORE_RELEASE(UNTUNED),

  L3_SCORE_LINEUP(UNTUNED),
  L3_SCORE_RELEASE(UNTUNED),

  L4_SCORE_LINEUP(UNTUNED),
  L4_SCORE_RELEASE(UNTUNED);

  public final double height;

  private ElevatorState(double height) {
    this.height = height;
  }

  private ElevatorState(ElevatorState state) {
    this.height = state.height;
  }
}
