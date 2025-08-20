package frc.robot.intake;

public enum IntakeState {
  UNTUNED(0.0),

  STOPPED(0.0),
  IDLE(UNTUNED),
  INTAKING(UNTUNED),
  OUTTAKING(UNTUNED),
  HANDOFF(UNTUNED),
  SCORING(UNTUNED);

  public final double volts;

  private IntakeState(IntakeState state) {
    this.volts = state.volts;
  }

  private IntakeState(double volts) {
    this.volts = volts;
  }
}
