package frc.robot.intake_deploy;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;

public enum DeployState {
  UNHOMED(0.0),
  HOMING(0.0),
  STOWED(108.0),
  FLOOR_INTAKE(-34),
  HANDOFF(113.0),
  L1_SCORE(70.0),

  UNJAM(0);

  private final DoubleSubscriber tunableAngle;

  private DeployState(double angle) {
    this.tunableAngle = DogLog.tunable("Deploy/State/" + name(), angle);
  }

  public double getAngle() {
    return tunableAngle.get();
  }
}
