package frc.robot.autos.auto_path_commands.red;

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
import frc.robot.util.trailblazer.AutoPoint;
import frc.robot.util.trailblazer.AutoSegment;
import frc.robot.util.trailblazer.Trailblazer;

public class RedLollipopLeftABAuto extends BaseAuto {
  public RedLollipopLeftABAuto(RobotManager robotManager, Trailblazer trailblazer) {
    super(robotManager, trailblazer);
  }

  @Override
  protected Pose2d getStartingPose() {
    return Points.START_R1_AND_B1.redPose;
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
                            new AutoPoint(new Pose2d(12.836, 1.085, Rotation2d.fromDegrees(90.0))),
                            new AutoPoint(new Pose2d(14.644, 2.240, Rotation2d.fromDegrees(90.0))),
                            new AutoPoint(
                                new Pose2d(15.034, 3.629, Rotation2d.fromDegrees(90.0)),
                                autoCommands.l4ApproachCommand(
                                    ReefPipe.PIPE_A, RobotScoringSide.LEFT))))),
                blocks.scoreL4(ReefPipe.PIPE_A, RobotScoringSide.LEFT),
                autoCommands.intakeLollipopCommand())),
        // LOLLIPOP 2 (MIDDLE)
        timing.time(
            "Piece 1",
            blocks.intakeLollipop(
                new Pose2d(15.7, 4.0, Rotation2d.fromDegrees(0))
                    .transformBy(AutoBlocks.LOLLIPOP_OFFSET)),
            blocks.scoreL4(ReefPipe.PIPE_B, RobotScoringSide.LEFT)),
        autoCommands.intakeLollipopCommand(),
        // LOLLIPOP 1
        timing.time(
            "Piece 2",
            blocks.intakeLollipop(
                new Pose2d(15.967, 2.658, Rotation2d.fromDegrees(-51.95))
                    .transformBy(AutoBlocks.LOLLIPOP_OFFSET)),
            blocks.scoreL2(ReefPipe.PIPE_A, RobotScoringSide.LEFT),
            autoCommands.intakeLollipopCommand()),
        // LOLLIPOP 3
        timing.time(
            "Piece 3",
            blocks.intakeLollipop(
                new Pose2d(15.967, 5.392, Rotation2d.fromDegrees(51.95))
                    .transformBy(AutoBlocks.LOLLIPOP_OFFSET)),
            blocks.scoreL2(ReefPipe.PIPE_B, RobotScoringSide.LEFT),
            Commands.none()));
  }
}
