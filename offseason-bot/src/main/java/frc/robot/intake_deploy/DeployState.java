package frc.robot.intake_deploy;

public enum DeployState {
  UNTUNED(0.0),
  UNHOMED(UNTUNED),
  OUTTAKE(UNTUNED),

  REHOME(UNTUNED),

  STOWED(UNTUNED),
  HANDOFF(UNTUNED),
  FLOOR_INTAKE(UNTUNED),
  L1_SCORE(UNTUNED);

  public final double angle;

  private DeployState(DeployState state) {
    this.angle = state.angle;
  }

  private DeployState(double angleDeg) {
    this.angle = angleDeg;
  }
}
