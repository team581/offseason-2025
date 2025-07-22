package frc.robot.autos;

import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.auto_align.ReefPipe;
import frc.robot.auto_align.ReefPipeLevel;
import frc.robot.auto_align.RobotScoringSide;
import frc.robot.config.RobotConfig;
import frc.robot.fms.FmsSubsystem;
import frc.robot.robot_manager.RobotManager;
import frc.robot.robot_manager.RobotState;
import frc.robot.util.PoseErrorTolerance;
import frc.robot.util.trailblazer.AutoPoint;
import frc.robot.util.trailblazer.AutoSegment;
import frc.robot.util.trailblazer.Trailblazer;
import frc.robot.util.trailblazer.constraints.AutoConstraintOptions;

public class AutoBlocks {
  /**
   * The tolerance used to determine when to end the little "backup & stow" motion we do after
   * scoring L4.
   */
  private static final PoseErrorTolerance AFTER_SCORE_POSITION_TOLERANCE =
      new PoseErrorTolerance(0.2, 25);

  private static final PoseErrorTolerance LOLLIPOP_APPROACH_TOLERANCE =
      new PoseErrorTolerance(0.6, 10);

  private static final PoseErrorTolerance SUPER_FAST_LOLLIPOP_APPROACH_TOLERANCE =
      new PoseErrorTolerance(0.8, 20);
  public static final PoseErrorTolerance APPROACH_REEF_TOLERANCE = new PoseErrorTolerance(0.3, 10);

  public static final Transform2d INTAKE_CORAL_GROUND_LINEUP_OFFSET =
      new Transform2d(-0.6, -0.9, Rotation2d.kZero);

  private static final Transform2d INTAKE_CORAL_GROUND_APPROACH_OFFSET =
      new Transform2d(0, Units.inchesToMeters(-60), Rotation2d.kZero);

  private static final Transform2d CENTER_LOLLIPOP_OFFSET =
      new Transform2d(0, Units.inchesToMeters(10), Rotation2d.kZero);
  private static final Transform2d APPROACH_LOLLIPOP_OFFSET =
      new Transform2d(0, Units.inchesToMeters(30), Rotation2d.kZero);

  public static final Transform2d LOLLIPOP_OFFSET =
      new Transform2d(
          0.0,
          -Units.inchesToMeters(RobotConfig.get().arm().inchesFromCenter()),
          Rotation2d.fromDegrees(90));
  public static final AutoConstraintOptions MAX_CONSTRAINTS =
      new AutoConstraintOptions(4.75, 57, 4.0, 30);
  public static final AutoConstraintOptions LOLLIPOP_RACE_CONSTRAINTS =
      MAX_CONSTRAINTS.withMaxLinearVelocity(5).withMaxLinearAcceleration(4.5);
  public static final AutoConstraintOptions BASE_CONSTRAINTS =
      new AutoConstraintOptions(4.0, 30, 2.5, 25);

  public static final AutoConstraintOptions CORAL_MAP_CONSTRAINTS =
      new AutoConstraintOptions(4.0, 10, 2.5, 10);
  private static final AutoConstraintOptions SCORING_CONSTRAINTS =
      BASE_CONSTRAINTS.withMaxLinearVelocity(3.0).withMaxLinearAcceleration(1.75);
  private static final AutoConstraintOptions L2_SCORING_CONSTRAINTS =
      BASE_CONSTRAINTS.withMaxLinearVelocity(3.3).withMaxLinearAcceleration(2.15);
  private static final AutoConstraintOptions LOLLIPOP_CONSTRAINTS =
      BASE_CONSTRAINTS.withMaxLinearAcceleration(1.0).withMaxLinearVelocity(2.8);

  private static final AutoConstraintOptions SUPER_FAST_LOLLIPOP_CONSTRAINTS =
      BASE_CONSTRAINTS.withMaxLinearAcceleration(3.0).withMaxLinearVelocity(4.5);

  public static final AutoConstraintOptions BASE_CONSTRAINTS_FOR_GROUND_AUTOS =
      new AutoConstraintOptions(3.75, 57, 1.75, 25);
  private static final AutoConstraintOptions SCORING_CONSTRAINTS_FOR_GROUND_AUTOS =
      BASE_CONSTRAINTS_FOR_GROUND_AUTOS.withMaxLinearAcceleration(1.25).withMaxLinearVelocity(3);

  private final Trailblazer trailblazer;
  private final RobotManager robotManager;
  private final AutoCommands autoCommands;

  public AutoBlocks(Trailblazer trailblazer, RobotManager robotManager, AutoCommands autoCommands) {
    this.trailblazer = trailblazer;
    this.robotManager = robotManager;
    this.autoCommands = autoCommands;
  }

  public Command scorePreloadL4(Pose2d startingPose, ReefPipe pipe, RobotScoringSide scoringSide) {
    return Commands.sequence(
        trailblazer.followSegment(
            new AutoSegment(
                BASE_CONSTRAINTS,
                new AutoPoint(
                    () ->
                        robotManager.autoAlign.getUsedScoringPose(
                            pipe, ReefPipeLevel.RAISING, RobotScoringSide.LEFT),
                    autoCommands
                        .preloadCoralCommand()
                        .andThen(autoCommands.l4ApproachCommand(pipe, scoringSide)),
                    BASE_CONSTRAINTS),
                new AutoPoint(
                    () ->
                        robotManager.autoAlign.getUsedScoringPose(
                            pipe, ReefPipeLevel.RAISING, RobotScoringSide.LEFT),
                    BASE_CONSTRAINTS),
                new AutoPoint(
                    () ->
                        robotManager.autoAlign.getUsedScoringPose(
                            pipe, ReefPipeLevel.RAISING, RobotScoringSide.LEFT),
                    autoCommands.l4ApproachCommand(pipe, scoringSide),
                    SCORING_CONSTRAINTS),
                // Actually align to score
                new AutoPoint(
                    () ->
                        robotManager.autoAlign.getUsedScoringPose(
                            pipe, ReefPipeLevel.L4, RobotScoringSide.LEFT),
                    SCORING_CONSTRAINTS))),
        //          .withDeadline(
        //
        // autoCommands.waitForAlignedForScore().andThen(autoCommands.l4LeftReleaseCommand())),
        trailblazer.followSegment(
            new AutoSegment(
                BASE_CONSTRAINTS,
                AFTER_SCORE_POSITION_TOLERANCE,
                // Start at the scoring position
                new AutoPoint(
                    () ->
                        robotManager.autoAlign.getUsedScoringPose(
                            pipe, ReefPipeLevel.RAISING, RobotScoringSide.LEFT),
                    Commands.waitSeconds(0.15).andThen(robotManager::stowRequest)),
                // Scoot back to the lineup position to finish the score
                new AutoPoint(
                    () ->
                        robotManager.autoAlign.getUsedScoringPose(
                            pipe, ReefPipeLevel.L4, RobotScoringSide.LEFT)))));
  }

  public Command scoreL4(ReefPipe pipe, RobotScoringSide scoringSide) {
    if (!robotManager.claw.getHasGP() && !robotManager.groundManager.hasCoral()) {
      DogLog.timestamp("AAAAAA");
      return Commands.none();
    }
    return Commands.sequence(
        Commands.runOnce(() -> robotManager.autoAlign.setAutoReefPipeOverride(pipe)),
        trailblazer
            .followSegment(
                new AutoSegment(
                    SCORING_CONSTRAINTS,
                    new AutoPoint(
                        () ->
                            pipe.getPose(
                                ReefPipeLevel.L4, FmsSubsystem.isRedAlliance(), scoringSide),
                        robotManager
                            .waitForStates(
                                RobotState.CLAW_CORAL,
                                RobotState.CORAL_L4_LEFT_APPROACH,
                                RobotState.CORAL_L4_RIGHT_APPROACH,
                                RobotState.STARTING_POSITION_CORAL)
                            .andThen(autoCommands.l4ApproachCommand(pipe, scoringSide)))),
                false)
            .withDeadline(autoCommands.waitForReleaseCommand()),
        trailblazer.followSegment(
            new AutoSegment(
                BASE_CONSTRAINTS,
                AFTER_SCORE_POSITION_TOLERANCE,
                new AutoPoint(
                    () ->
                        pipe.getPose(
                            ReefPipeLevel.BACK_AWAY, FmsSubsystem.isRedAlliance(), scoringSide)))));
  }

  public Command scoreL4AfterGroundIntake(ReefPipe pipe, RobotScoringSide scoringSide) {
    if (!robotManager.claw.getHasGP() && !robotManager.groundManager.hasCoral()) {
      return Commands.none();
    }
    return Commands.sequence(
        Commands.runOnce(() -> robotManager.autoAlign.setAutoReefPipeOverride(pipe)),
        trailblazer
            .followSegment(
                new AutoSegment(
                    SCORING_CONSTRAINTS_FOR_GROUND_AUTOS,
                    new AutoPoint(
                        () ->
                            pipe.getPose(
                                ReefPipeLevel.L4, FmsSubsystem.isRedAlliance(), scoringSide),
                        robotManager
                            .waitForStates(
                                RobotState.CLAW_CORAL,
                                RobotState.CORAL_L4_LEFT_APPROACH,
                                RobotState.CORAL_L4_RIGHT_APPROACH,
                                RobotState.STARTING_POSITION_CORAL)
                            .andThen(autoCommands.l4ApproachCommand(pipe, scoringSide)))),
                false)
            .withDeadline(autoCommands.waitForReleaseCommand()),
        trailblazer.followSegment(
            new AutoSegment(
                BASE_CONSTRAINTS_FOR_GROUND_AUTOS,
                AFTER_SCORE_POSITION_TOLERANCE,
                new AutoPoint(
                    () ->
                        pipe.getPose(
                            ReefPipeLevel.BACK_AWAY, FmsSubsystem.isRedAlliance(), scoringSide)))));
  }

  public Command intakeCoralGroundPoints(
      Pose2d lineup, Pose2d approachToIntake, Points intakingPoint) {
    return autoCommands
        .groundIntakeToL4Command()
        .alongWith(
            trailblazer
                .followSegment(
                    new AutoSegment(
                        BASE_CONSTRAINTS_FOR_GROUND_AUTOS,
                        new AutoPoint(lineup),
                        new AutoPoint(approachToIntake),
                        new AutoPoint(intakingPoint::getPose)),
                    true)
                .repeatedly()
                .withDeadline(autoCommands.waitForIntakeDone()));
  }

  public Command intakeCoralPath(Pose2d lineup, Pose2d approachToIntake, Pose2d intakingPoint) {
    return autoCommands
        .groundIntakeToL4Command()
        .alongWith(
            trailblazer
                .followSegment(
                    new AutoSegment(
                        BASE_CONSTRAINTS_FOR_GROUND_AUTOS,
                        new AutoPoint(lineup),
                        new AutoPoint(approachToIntake),
                        new AutoPoint(
                            () -> robotManager.coralMap.getBestCoralPose().orElse(intakingPoint),
                            Commands.runOnce(
                                () -> {
                                  autoCommands.groundIntakeToL4Command();
                                }))),
                    true)
                .repeatedly()
                .withDeadline(autoCommands.waitForIntakeDone()));
  }

  public Command scoreL3(ReefPipe pipe, RobotScoringSide scoringSide, Command onFinish) {
    if (!robotManager.claw.getHasGP() && !robotManager.groundManager.hasCoral()) {
      return Commands.none();
    }
    return Commands.sequence(
        trailblazer
            .followSegment(
                new AutoSegment(
                    SCORING_CONSTRAINTS,
                    new AutoPoint(
                        () -> robotManager.autoAlign.getUsedScoringPose(pipe, scoringSide),
                        Commands.runOnce(() -> robotManager.autoAlign.setAutoReefPipeOverride(pipe))
                            .andThen(
                                robotManager.waitForStates(
                                    RobotState.CLAW_CORAL,
                                    RobotState.CORAL_L3_LEFT_APPROACH,
                                    RobotState.CORAL_L3_RIGHT_APPROACH,
                                    RobotState.STARTING_POSITION_CORAL))
                            .andThen(autoCommands.l3ApproachCommand(scoringSide)))),
                false)
            .withDeadline(autoCommands.waitForReleaseCommand().withTimeout(3)),
        trailblazer.followSegment(
            new AutoSegment(
                BASE_CONSTRAINTS,
                AFTER_SCORE_POSITION_TOLERANCE,
                new AutoPoint(
                    () ->
                        pipe.getPose(
                            ReefPipeLevel.BACK_AWAY, FmsSubsystem.isRedAlliance(), scoringSide),
                    Commands.waitSeconds(0.15).andThen(onFinish)))));
  }

  public Command scoreL3(ReefPipe pipe, RobotScoringSide scoringSide) {
    return scoreL3(pipe, scoringSide, Commands.runOnce(robotManager::stowRequest));
  }

  public Command scoreL2(ReefPipe pipe, RobotScoringSide scoringSide) {
    if (!robotManager.claw.getHasGP() && !robotManager.groundManager.hasCoral()) {
      return Commands.none();
    }
    return Commands.sequence(
        trailblazer
            .followSegment(
                new AutoSegment(
                    L2_SCORING_CONSTRAINTS,
                    new AutoPoint(
                        () -> robotManager.autoAlign.getUsedScoringPose(pipe, scoringSide),
                        Commands.runOnce(() -> robotManager.autoAlign.setAutoReefPipeOverride(pipe))
                            .andThen(
                                robotManager.waitForStates(
                                    RobotState.CLAW_CORAL,
                                    RobotState.CORAL_L2_LEFT_APPROACH,
                                    RobotState.CORAL_L2_RIGHT_APPROACH,
                                    RobotState.STARTING_POSITION_CORAL))
                            .andThen(autoCommands.l2LineupCommand(scoringSide)))),
                false)
            .withDeadline(autoCommands.waitForReleaseCommand().withTimeout(3)),
        trailblazer.followSegment(
            new AutoSegment(
                BASE_CONSTRAINTS,
                AFTER_SCORE_POSITION_TOLERANCE,
                new AutoPoint(
                    () ->
                        pipe.getPose(
                            ReefPipeLevel.BACK_AWAY, FmsSubsystem.isRedAlliance(), scoringSide)))));
  }

  public Command intakeLollipop(Pose2d defaultIntakingPoint) {
    return Commands.sequence(
            Commands.runOnce(
                () -> {
                  robotManager.coralMap.clearLollipop();
                  robotManager.swerve.snapsDriveRequest(
                      defaultIntakingPoint.getRotation().getDegrees());
                }),
            autoCommands.intakeLollipopCommand(),
            trailblazer
                .followSegment(
                    new AutoSegment(
                        BASE_CONSTRAINTS,
                        new AutoPoint(defaultIntakingPoint.transformBy(APPROACH_LOLLIPOP_OFFSET))),
                    false)
                .withDeadline(autoCommands.waitForElevatorAndArmNearLollipop()),
            trailblazer.followSegment(
                new AutoSegment(
                    BASE_CONSTRAINTS,
                    LOLLIPOP_APPROACH_TOLERANCE,
                    new AutoPoint(defaultIntakingPoint.transformBy(APPROACH_LOLLIPOP_OFFSET))),
                true),
            trailblazer.followSegment(
                new AutoSegment(
                    LOLLIPOP_CONSTRAINTS,
                    new AutoPoint(
                        () ->
                            new Pose2d(
                                robotManager
                                    .coralMap
                                    .getLollipopIntakePose()
                                    .orElse(defaultIntakingPoint)
                                    .transformBy(CENTER_LOLLIPOP_OFFSET)
                                    .getTranslation(),
                                defaultIntakingPoint.getRotation()),
                        BASE_CONSTRAINTS),
                    new AutoPoint(
                        () ->
                            new Pose2d(
                                robotManager
                                    .coralMap
                                    .getLollipopIntakePose()
                                    .orElse(defaultIntakingPoint)
                                    .getTranslation(),
                                defaultIntakingPoint.getRotation()))),
                false))
        .withDeadline(autoCommands.waitForLollipopIntakeDone())
        .andThen(Commands.runOnce(() -> robotManager.swerve.normalDriveRequest()));
  }

  public Command intakeLollipopSuperFast(Pose2d defaultIntakingPoint) {
    return Commands.sequence(
            Commands.runOnce(
                () -> {
                  robotManager.coralMap.clearLollipop();
                  robotManager.swerve.snapsDriveRequest(
                      defaultIntakingPoint.getRotation().getDegrees());
                }),
            autoCommands.intakeLollipopCommand(),
            trailblazer
                .followSegment(
                    new AutoSegment(
                        MAX_CONSTRAINTS,
                        new AutoPoint(defaultIntakingPoint.transformBy(APPROACH_LOLLIPOP_OFFSET))),
                    false)
                .withDeadline(autoCommands.waitForElevatorAndArmNearLollipop()),
            trailblazer.followSegment(
                new AutoSegment(
                    MAX_CONSTRAINTS,
                    LOLLIPOP_APPROACH_TOLERANCE,
                    new AutoPoint(defaultIntakingPoint.transformBy(APPROACH_LOLLIPOP_OFFSET))),
                true),
            trailblazer.followSegment(
                new AutoSegment(
                    SUPER_FAST_LOLLIPOP_CONSTRAINTS,
                    new AutoPoint(
                        () ->
                            new Pose2d(
                                robotManager
                                    .coralMap
                                    .getLollipopIntakePose()
                                    .orElse(defaultIntakingPoint)
                                    .transformBy(CENTER_LOLLIPOP_OFFSET)
                                    .getTranslation(),
                                defaultIntakingPoint.getRotation()),
                        MAX_CONSTRAINTS),
                    new AutoPoint(
                        () ->
                            new Pose2d(
                                robotManager
                                    .coralMap
                                    .getLollipopIntakePose()
                                    .orElse(defaultIntakingPoint)
                                    .getTranslation(),
                                defaultIntakingPoint.getRotation()))),
                false))
        .withDeadline(autoCommands.waitForLollipopIntakeDone())
        .andThen(Commands.runOnce(() -> robotManager.swerve.normalDriveRequest()));
  }

  public Command intakeGroundForL4(Pose2d defaultIntakingPose) {
    return trailblazer
        .followSegment(
            new AutoSegment(
                CORAL_MAP_CONSTRAINTS,
                new AutoPoint(
                    () -> robotManager.coralMap.getBestCoralPose().orElse(defaultIntakingPose),
                    Commands.runOnce(
                        () -> {
                          autoCommands.groundIntakeToL4Command();
                        }))),
            false)
        .withDeadline(autoCommands.waitForIntakeDone());
  }
}
