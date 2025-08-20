package frc.robot.singulator;

public enum SingulatorState {
  UNTUNED(0.0),

  IDLE(UNTUNED),
  INTAKING(UNTUNED),
  OUTTAKING(UNTUNED),
  SCORING(UNTUNED),
  HANDOFF(UNTUNED);

  public final double volts;
  private SingulatorState(SingulatorState state) {
    this.volts = state.volts;
  }
  private SingulatorState(double volts) {
    this.volts = volts;
  }
}
