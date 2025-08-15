package frc.robot.util;

import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

import org.junit.jupiter.api.Test;

public class VelocityDetectorTest {
  @Test
  void testDetectionSequence() {
    var detector = new VelocityDetector(10, Double.MAX_VALUE, 0.0);

    // Velocity is below the min threshold
    assertFalse(detector.hasGamePiece(0, 5));
    // Velocity is above the min threshold
    assertFalse(detector.hasGamePiece(15, 5));
    // Velocity has reached the min, and now is below max
    assertTrue(detector.hasGamePiece(3, 5));
    detector.reset();
    // Velocity is below max, but detector was reset
    assertFalse(detector.hasGamePiece(3, 5));
  }
}
