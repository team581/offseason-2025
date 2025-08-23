package frc.robot.vision.results;

public class GamePieceResult {
  private double tx;
  private double ty;
  private double timestamp;

  public GamePieceResult() {
    this.tx = 0.0;
    this.ty = 0.0;
    this.timestamp = 0.0;
  }

  public double tx() {
    return tx;
  }

  public double ty() {
    return ty;
  }

  public double timestamp() {
    return timestamp;
  }

  public void update(double tx, double ty, double timestamp) {
    this.tx = tx;
    this.ty = ty;
    this.timestamp = timestamp;
  }
}
