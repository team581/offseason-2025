package com.team581.math;

import edu.wpi.first.math.MathUtil;

public final class ControllerHelpers {
  private static final double LOWER_DEADBAND = 0.05;
  private static final double UPPER_DEADBAND = 0.95;

  /**
   * Given the raw X and Y values from a joystick, calculate the magnitude of the vector, apply
   * deadbands, and then apply an exponent.
   *
   * @param joystickX The X value from the joystick.
   * @param joystickY The Y value from the joystick.
   * @param exponent The exponent to apply to the magnitude.
   * @return The magnitude of the vector, with deadbands and exponent applied.
   */
  public static double getJoystickMagnitude(double joystickX, double joystickY, double exponent) {
    var rawMagnitude = Math.hypot(joystickX, joystickY);

    var deadbandedLower = MathUtil.applyDeadband(rawMagnitude, LOWER_DEADBAND, 1);
    var deadbandedUpper =
        MathUtil.interpolate(
            0, Math.signum(deadbandedLower), Math.abs(deadbandedLower) / UPPER_DEADBAND);

    return Math.pow(deadbandedUpper, exponent);
  }

  private ControllerHelpers() {}
}
