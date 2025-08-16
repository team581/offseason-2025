package frc.robot.util.trailblazer.followers;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;

/** Generates swerve setpoints using robot pose and target pose. */
public interface PathFollower {
  /**
   * Given the current pose of the robot and the target pose, calculate the velocity the robot
   * should drive at to get there.
   *
   * @param currentPose The current pose of the robot.
   * @param targetPose The target pose to drive to.
   * @return The desired robot velocity to drive at.
   */
  public ChassisSpeeds calculateSpeeds(Pose2d currentPose, Pose2d targetPose);
}
