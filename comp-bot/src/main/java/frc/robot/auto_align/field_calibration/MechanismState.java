package frc.robot.auto_align.field_calibration;

enum MechanismState {
  TOO_LOW("too low"),
  TOO_HIGH("too high"),
  OK("ok");

  public String label;

  MechanismState(String label) {
    this.label = label;
  }
}
