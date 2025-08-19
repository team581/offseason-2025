package frc.robot.autos.auto_path_commands.blue;

import com.team581.trailblazer.AutoPoint;
import com.team581.trailblazer.AutoSegment;
import com.team581.trailblazer.Trailblazer;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto_align.ReefPipe;
import frc.robot.auto_align.RobotScoringSide;
import frc.robot.autos.AutoBlocks;
import frc.robot.autos.BaseAuto;
import frc.robot.autos.Points;
import frc.robot.robot_manager.RobotManager;

public class BlueLollipopLeftABAuto extends BaseAuto {
  public BlueLollipopLeftABAuto(RobotManager robotManager, Trailblazer trailblazer) {
    super(robotManager, trailblazer);
  }

  @Override
  protected Pose2d getStartingPose() {
    return Points.START_R1_AND_B1.bluePose;
  }

  @Override
  protected Command createAutoCommand() {
    return Commands.sequence(
        Commands.parallel(
            autoCommands.homeDeployCommand(),
            autoCommands.preloadCoralCommand(),
            timing.time(
                "Preload",
                timing.time(
                    "Preload race",
                    trailblazer.followSegment(
                        new AutoSegment(
                            AutoBlocks.LOLLIPOP_RACE_CONSTRAINTS,
                            AutoBlocks.APPROACH_REEF_TOLERANCE,
                            new AutoPoint(new Pose2d(4.714, 6.965, Rotation2d.fromDegrees(-90))),
                            new AutoPoint(new Pose2d(2.906, 5.81, Rotation2d.fromDegrees(-90))),
                            new AutoPoint(
                                new Pose2d(2.516, 4.421, Rotation2d.fromDegrees(-90)),
                                autoCommands.l4ApproachCommand(
                                    ReefPipe.PIPE_A, RobotScoringSide.LEFT))))),
                blocks.scoreL4(ReefPipe.PIPE_A, RobotScoringSide.LEFT),
                autoCommands.intakeLollipopCommand())),
        // LOLLIPOP 2 (MIDDLE)
        timing.time(
            "Piece 1",
            blocks.intakeLollipop(
                new Pose2d(1.85, 4.05, Rotation2d.fromDegrees(180))
                    .transformBy(AutoBlocks.LOLLIPOP_OFFSET)),
            blocks.scoreL4(ReefPipe.PIPE_B, RobotScoringSide.LEFT)),
        autoCommands.intakeLollipopCommand(),
        // LOLLIPOP 1
        timing.time(
            "Piece 2",
            blocks.intakeLollipop(
                new Pose2d(1.583, 5.392, Rotation2d.fromDegrees(128.05))
                    .transformBy(AutoBlocks.LOLLIPOP_OFFSET)),
            blocks.scoreL2(ReefPipe.PIPE_A, RobotScoringSide.LEFT),
            autoCommands.intakeLollipopCommand()),
        // LOLLIPOP 3
        timing.time(
            "Piece 3",
            blocks.intakeLollipop(
                new Pose2d(1.583, 2.658, Rotation2d.fromDegrees(-128.05))
                    .transformBy(AutoBlocks.LOLLIPOP_OFFSET)),
            blocks.scoreL2(ReefPipe.PIPE_B, RobotScoringSide.LEFT),
            Commands.none()));
  }
}
