package frc.robot.autos.auto_path_commands.blue;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.autos.BaseAuto;
import frc.robot.autos.Points;
import frc.robot.robot_manager.RobotManager;
import frc.robot.util.trailblazer.AutoPoint;
import frc.robot.util.trailblazer.AutoSegment;
import frc.robot.util.trailblazer.Trailblazer;
import frc.robot.util.trailblazer.constraints.AutoConstraintOptions;

public class BlueStraightLineAuto extends BaseAuto {
  private static final AutoConstraintOptions CONSTRAINTS =
      new AutoConstraintOptions(4.75, 71.5, 8.5, 35.2);

  public BlueStraightLineAuto(RobotManager robotManager, Trailblazer trailblazer) {
    super(robotManager, trailblazer);
  }

  @Override
  protected Pose2d getStartingPose() {
    return Points.START_R1_AND_B1_FORWARD.bluePose;
  }

  @Override
  protected Command createAutoCommand() {
    return Commands.sequence(
        trailblazer.followSegment(
            new AutoSegment(
                CONSTRAINTS,
                new AutoPoint(Points.START_R1_AND_B1_FORWARD.bluePose),
                new AutoPoint(new Pose2d(5.626, 7.292, Rotation2d.kZero)),
                new AutoPoint(new Pose2d(2.55, 7.292, Rotation2d.kZero)))));
  }
}
