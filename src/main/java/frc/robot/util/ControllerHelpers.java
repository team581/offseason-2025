package frc.robot.util;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;

public final class ControllerHelpers {
  public static double deadbandJoystickValue(double joystickValue, double deadband) {
    return MathUtil.applyDeadband(joystickValue, deadband, 1);
  }

  public static Translation2d fromCircularDiscCoordinates(double x, double y) {
    var rawinputs = new Translation2d(x, y);
    var magnitude = rawinputs.getNorm();
    if (Math.abs(magnitude) <= 0.05) {
      magnitude = 0;
    } else {
      if (Math.abs(magnitude) >= 0.9) {
        magnitude = Math.copySign(1, magnitude);
      }
    }
    return new Translation2d(magnitude, rawinputs.getAngle());
  }

  private ControllerHelpers() {}
}
