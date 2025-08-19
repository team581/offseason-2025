package frc.robot.util.scheduling;

import com.team581.util.scheduling.SubsystemPriorityBase;

public enum SubsystemPriority implements SubsystemPriorityBase {
  // 20-30 is for manager subsystems
  AUTOS(30),
  ROBOT_MANAGER(29),
  GROUND_MANAGER(28),

  // 10-19 is for sensor subsystems
  // Auto align inputs run after localization pose is updated
  AUTO_ALIGN(12),
  LOCALIZATION(11),
  FMS(10),
  IMU(10),
  // Vision inputs run before localization so that it has fresh vision data for pose estimator
  VISION(10),

  // 0-9 is for actuator subsystems
  SWERVE(0),
  DEPLOY(0),
  INTAKE(0),
  CLAW(0),
  ELEVATOR(0),
  ARM(0),
  CLIMBER(0),
  LIGHTS(0),
  RUMBLE_CONTROLLER(0);

  public final int value;

  private SubsystemPriority(int priority) {
    this.value = priority;
  }

  @Override
  public int getValue() {
    return value;
  }
}
