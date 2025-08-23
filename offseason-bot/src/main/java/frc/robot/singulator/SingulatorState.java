package frc.robot.singulator;

public enum SingulatorState {
  UNTUNED(0.0),

  UNJAM_RIGHT_ONLY(0.0, 0.0),
  UNJAM_LEFT_ONLY(0.0, 0.0),

  STOPPED(0.0),
  IDLE(UNTUNED),
  INTAKING(UNTUNED),
  OUTTAKING(UNTUNED),
  HANDOFF(UNTUNED);

  public final double voltsLeft;
  public final double voltsRight;

  private SingulatorState(SingulatorState state) {
    this.voltsLeft = state.voltsLeft;
    this.voltsRight = state.voltsRight;
  }

  private SingulatorState(double voltsBoth) {
    this.voltsLeft = voltsBoth;
    this.voltsRight = voltsBoth;
  }

  private SingulatorState(double voltsLeft, double voltsRight) {
    this.voltsLeft = voltsLeft;
    this.voltsRight = voltsRight;
  }
}
