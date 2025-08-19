package com.team581.math;

import static org.junit.jupiter.api.Assertions.assertEquals;

import org.junit.jupiter.api.Test;

public class ControllerHelpersTest {
  @Test
  void joystickMagnitudeAllZeroTest() {
    var actual = ControllerHelpers.getJoystickMagnitude(0, 0, 2);
    assertEquals(0, actual);
  }

  @Test
  void joystickMagnitudeDeadbandPositiveTest() {
    var actual = ControllerHelpers.getJoystickMagnitude(0.03, 0.03, 2);
    assertEquals(0, actual);
  }

  @Test
  void joystickMagnitudeDeadbandNegativeTest() {
    var actual = ControllerHelpers.getJoystickMagnitude(-0.03, -0.03, 2);
    assertEquals(0, actual);
  }

  @Test
  void joystickMagnitudeUpperDeadbandPositiveTest() {
    var actual = ControllerHelpers.getJoystickMagnitude(0.97, 0.03, 2);
    assertEquals(1, actual);
  }
}
