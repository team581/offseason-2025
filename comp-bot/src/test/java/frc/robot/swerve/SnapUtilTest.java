package frc.robot.swerve;

import static org.junit.jupiter.api.Assertions.assertEquals;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import org.junit.jupiter.api.Test;

class SnapUtilTest {
  @Test
  void getNearestReefAngleTest() {
    var pose = new Pose2d(11.807, 5.586, Rotation2d.fromDegrees(-81));
    var result = SnapUtil.getNearestReefAngle(pose);

    assertEquals(300, result);
  }
}
