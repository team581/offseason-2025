package frc.robot.arm;

import dev.doglog.DogLog;
import edu.wpi.first.networktables.DoubleSubscriber;

public enum ArmState {
  /**
   * @deprecated This is a placeholder state.
   */
  @Deprecated
  UNTUNED(90),

  PRE_MATCH_HOMING(0),

  UNJAM(UNTUNED),
  // Idle positions
  HOLDING_UPRIGHT(90),
  STOWED_INWARD(90),

  // Left coral positions
  CORAL_SCORE_LEFT_LINEUP_L2(180 - 30.0),
  CORAL_SCORE_LEFT_RELEASE_L2(180 - 5.0),

  CORAL_SCORE_LEFT_LINEUP_L3(180 - 30.0),
  CORAL_SCORE_LEFT_RELEASE_L3(180 - 5.0),

  CORAL_SCORE_LEFT_LINEUP_L4(180.0 - 50.0),
  CORAL_SCORE_LEFT_RELEASE_L4(180.0 - 10.0),

  // Right coral positions
  CORAL_SCORE_RIGHT_LINEUP_L1(35.0),
  CORAL_SCORE_RIGHT_RELEASE_L1(35.0),

  CORAL_SCORE_RIGHT_LINEUP_L2(30.0),
  CORAL_SCORE_RIGHT_RELEASE_L2(5.0),

  CORAL_SCORE_RIGHT_LINEUP_L3(30.0),
  CORAL_SCORE_RIGHT_RELEASE_L3(5.0),

  CORAL_SCORE_RIGHT_LINEUP_L4(50.0),
  CORAL_SCORE_RIGHT_RELEASE_L4(10.0),

  // Handoffs
  CORAL_HANDOFF(-90.0),

  // Algae positions
  ALGAE_INTAKE_FLOOR(-43.0),

  ALGAE_INTAKE_LEFT_L2(180.0),
  ALGAE_INTAKE_LEFT_L3(180.0),

  ALGAE_INTAKE_RIGHT_L2(0.0),
  ALGAE_INTAKE_RIGHT_L3(0.0),

  ALGAE_NET_LEFT(130),

  ALGAE_NET_RIGHT(50),

  ALGAE_PROCESSOR(-20.0),

  ALGAE_OUTTAKE(UNTUNED),

  COLLISION_AVOIDANCE(UNTUNED),
  // TODO: Setting the arm to 60 can lead to false positives when coming from net scoring to algae
  // ground intake. The robot will be partially done with the motion to go to handoff, and the
  // nearest waypoint is climbing, which is used as the path start when the algae intake request is
  // received.
  // CLIMBING(60.0),
  CLIMBING(0.0),

  SPIN_TO_WIN(90),

  ALGAE_FLING_WAIT(UNTUNED),
  ALGAE_FLING_SWING(UNTUNED),

  // For auto
  LOLLIPOP_CORAL_INTAKE_PUSH(-22),
  LOLLIPOP_CORAL_INTAKE_INTAKE(-9);

  private final double defaultAngle;
  private final DoubleSubscriber tunableAngle;

  ArmState(double angle) {
    this.defaultAngle = angle;
    this.tunableAngle = DogLog.tunable("Arm/State/" + name(), angle);
  }

  ArmState(ArmState other) {
    this(other.defaultAngle);
  }

  public double getAngle() {
    return tunableAngle.get();
  }
}
