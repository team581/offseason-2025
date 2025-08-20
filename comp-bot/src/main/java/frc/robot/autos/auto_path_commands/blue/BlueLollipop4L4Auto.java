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

public class BlueLollipop4L4Auto extends BaseAuto {
  public BlueLollipop4L4Auto(RobotManager robotManager, Trailblazer trailblazer) {
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
                            new AutoPoint(new Pose2d(3.734, 6.669, Rotation2d.fromDegrees(-150))),
                            new AutoPoint(
                                new Pose2d(3.85, 5.55, Rotation2d.fromDegrees(-150)),
                                autoCommands.l4ApproachCommand(
                                    ReefPipe.PIPE_K, RobotScoringSide.LEFT))))),
                blocks.scoreL4(ReefPipe.PIPE_K, RobotScoringSide.LEFT),
                autoCommands.intakeLollipopCommand())),
        // LOLLIPOP 1
        timing.time(
            "Piece 1",
            blocks.intakeLollipop(
                new Pose2d(1.783, 5.682, Rotation2d.fromDegrees(161.814))
                    .transformBy(AutoBlocks.LOLLIPOP_OFFSET)),
            blocks.scoreL4(ReefPipe.PIPE_L, RobotScoringSide.LEFT),
            autoCommands.intakeLollipopCommand()),
        // LOLLIPOP 2 (MIDDLE)
        timing.time(
            "Piece 2",
            blocks.intakeLollipop(
                new Pose2d(1.769, 4.241, Rotation2d.fromDegrees(-158.929))
                    .transformBy(AutoBlocks.LOLLIPOP_OFFSET)),
            blocks.scoreL4(ReefPipe.PIPE_A, RobotScoringSide.LEFT)),
        autoCommands.intakeLollipopCommand(),
        // LOLLIPOP 3
        timing.time(
            "Piece 3",
            blocks.intakeLollipop(
                new Pose2d(1.583, 2.658, Rotation2d.fromDegrees(-128.05))
                    .transformBy(AutoBlocks.LOLLIPOP_OFFSET)),
            blocks.scoreL4(ReefPipe.PIPE_B, RobotScoringSide.LEFT),
            Commands.none()));
  }
}
