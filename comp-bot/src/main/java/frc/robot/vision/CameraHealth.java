package frc.robot.vision;

public enum CameraHealth {
  GOOD,
  OFFLINE,
  NO_TARGETS;

  public static CameraHealth combine(CameraHealth a, CameraHealth b) {
    if (a == b) {
      return a;
    }
    if (a == CameraHealth.GOOD || b == CameraHealth.GOOD) {
      return CameraHealth.GOOD;
    }
    return CameraHealth.NO_TARGETS;
  }

  public static CameraHealth combine(CameraHealth a, CameraHealth b, CameraHealth c) {
    return combine(combine(a, b), c);
  }
}
