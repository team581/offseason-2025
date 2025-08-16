package frc.robot.util;

import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.wpilibj.Timer;

public class VelocityDetector {
  private final double minVelocity;
  private final double minVelocityTimeout;
  private final Timer timeout = new Timer();
  private final Debouncer debouncer;
  private boolean hasSeenMinVelocity = false;

  public VelocityDetector(double minVelocity, double minVelocityTimeout, double debounceTime) {
    this.minVelocity = minVelocity;
    this.minVelocityTimeout = minVelocityTimeout;
    this.debouncer = new Debouncer(debounceTime, DebounceType.kRising);
    timeout.start();
  }

  public void reset() {
    hasSeenMinVelocity = false;
    timeout.reset();
  }

  /**
   * Returns whether the motor is holding a game piece.
   *
   * @param motorVelocity Current motor velocity.
   */
  public boolean hasGamePiece(double motorVelocity, double maxVelocity) {
    hasSeenMinVelocity =
        hasSeenMinVelocity
            || timeout.hasElapsed(minVelocityTimeout)
            || motorVelocity >= minVelocity;

    return hasSeenMinVelocity && debouncer.calculate(motorVelocity <= maxVelocity);
  }
}
