package frc.robot.auto_align.field_calibration;

record Summary(MechanismState elevator, MechanismState arm, boolean align, boolean heading) {
  public String format() {
    var alignLabel = align ? "ok" : "bad";
    var headingLabel = heading ? "ok" : "bad";
    return "Elevator "
        + elevator.label
        + ", arm "
        + arm.label
        + ", align "
        + alignLabel
        + ", heading "
        + headingLabel;
  }

  public boolean isOk() {
    return elevator == MechanismState.OK && arm == MechanismState.OK && align && heading;
  }
}
