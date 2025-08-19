package frc.robot.vision.results;

import com.google.errorprone.annotations.CanIgnoreReturnValue;
import com.team581.util.ReusableOptional;
import edu.wpi.first.math.Vector;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.numbers.N3;

public class OptionalTagResult extends ReusableOptional<TagResult> {
  public OptionalTagResult() {
    super(new TagResult());
  }

  @CanIgnoreReturnValue
  public OptionalTagResult update(Pose2d pose, double timestamp, Vector<N3> standardDevs) {
    this.value.update(pose, timestamp, standardDevs);
    this.isPresent = true;

    return this;
  }

  @CanIgnoreReturnValue
  public OptionalTagResult empty() {
    this.value.update(null, 0, null);
    this.isPresent = false;
    return this;
  }
}
