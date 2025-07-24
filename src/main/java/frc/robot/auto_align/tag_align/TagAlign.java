package frc.robot.auto_align.tag_align;

import com.google.common.collect.ImmutableList;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.auto_align.ReefPipe;
import frc.robot.auto_align.ReefPipeLevel;
import frc.robot.auto_align.ReefState;
import frc.robot.auto_align.RobotScoringSide;
import frc.robot.config.FeatureFlags;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.MathHelpers;
import frc.robot.util.kinematics.PolarChassisSpeeds;
import frc.robot.util.trailblazer.constraints.AutoConstraintOptions;
import java.util.Comparator;
import java.util.Optional;

public class TagAlign {
  private static final ImmutableList<ReefPipe> ALL_REEF_PIPES =
      ImmutableList.copyOf(ReefPipe.values());

  private static final PIDController ROTATION_CONTROLLER = new PIDController(6.0, 0.0, 0.0);

  private static final DoubleSubscriber TRANSLATION_GOOD_THRESHOLD =
      DogLog.tunable("AutoAlign/IsAlignedTranslation", Units.inchesToMeters(1.0));
        private static final DoubleSubscriber ROTATION_GOOD_THRESHOLD =
            DogLog.tunable("AutoAlign/IsAlignedRotation", 3.0);

      private static final DoubleSubscriber NEED_TO_MOVE_TRANSLATION_THRESHOLD =
      DogLog.tunable("AutoAlign/NeedMoveTranslation", Units.inchesToMeters(1.5));
      private static final DoubleSubscriber NEED_TO_MOVE_ROTATION_THRESHOLD =
      DogLog.tunable("AutoAlign/NeedMoveRotation", 3.0);

      private static final DoubleSubscriber IN_RANGE_TRANSLATION_THRESHOLD =
      DogLog.tunable("AutoAlign/InRangeTranslation", Units.inchesToMeters(1.5));
      private static final DoubleSubscriber IN_RANGE_ROTATION_THRESHOLD =
      DogLog.tunable("AutoAlign/InRangeRotation", 3.0);


      private static final double MAX_SPEED = 2.0;
  private static final PIDController VELOCITY_CONTROLLER = new PIDController(3.7, 0.0, 0.0);
  private static final double PIPE_SWITCH_TIMEOUT = 0.5;

  private final AlignmentCostUtil alignmentCostUtil;
  private final LocalizationSubsystem localization;
  private final ReefState reefState = new ReefState();

  private ReefPipeLevel pipeLevel = ReefPipeLevel.RAISING;
  private ReefPipeLevel preferedScoringLevel = ReefPipeLevel.L4;
  private RobotScoringSide robotScoringSide = RobotScoringSide.RIGHT;
  private Optional<ReefPipe> reefPipeOverride = Optional.empty();
  private double rawControllerXValue = 0.0;
  private double rawControllerYValue = 0.0;
  private double lastPipeSwitchTimestamp = 0.0;
  private boolean aligned = false;

  private static final DoubleSubscriber FEED_FORWARD = DogLog.tunable("AutoAlign/FeedForward", 0.0);

  private boolean pipeSwitchActive = false;

  public TagAlign(SwerveSubsystem swerve, LocalizationSubsystem localization) {
    this.localization = localization;
    alignmentCostUtil = new AlignmentCostUtil(localization, swerve, reefState, robotScoringSide);
    ROTATION_CONTROLLER.enableContinuousInput(-Math.PI, Math.PI);
    ROTATION_CONTROLLER.setTolerance(0.01);
  }

  public void setLevel(ReefPipeLevel level, ReefPipeLevel preferredLevel, RobotScoringSide side) {
    this.robotScoringSide = side;
    alignmentCostUtil.setSide(robotScoringSide);
    this.pipeLevel = level;
    this.preferedScoringLevel = preferredLevel;
  }

  public void setPipeOveride(ReefPipe pipe) {
    this.reefPipeOverride = Optional.of(pipe);
  }

  public void setControllerValues(double controllerXValue, double controllerYValue) {
    rawControllerXValue = controllerXValue;
    rawControllerYValue = controllerYValue;
    checkControllerForSwitch();
  }

  private void checkControllerForSwitch() {
    if (!DriverStation.isTeleop()) {
      return;
    }
    if (pipeSwitchActive
        && (Timer.getFPGATimestamp() > (lastPipeSwitchTimestamp + PIPE_SWITCH_TIMEOUT))
        && rawControllerXValue == 0.0) {
      pipeSwitchActive = false;
    }
    if (pipeSwitchActive) {
      return;
    }
    if ((Math.hypot(rawControllerXValue, rawControllerYValue) > 0.5)) {
      var storedPipe = getBestPipe();
      pipeSwitchActive = true;
      lastPipeSwitchTimestamp = Timer.getFPGATimestamp();
      ReefPipe partnerPipe =
          switch (storedPipe) {
            case PIPE_A -> ReefPipe.PIPE_B;
            case PIPE_B -> ReefPipe.PIPE_A;
            case PIPE_C -> ReefPipe.PIPE_D;
            case PIPE_D -> ReefPipe.PIPE_C;
            case PIPE_E -> ReefPipe.PIPE_F;
            case PIPE_F -> ReefPipe.PIPE_E;
            case PIPE_G -> ReefPipe.PIPE_H;
            case PIPE_H -> ReefPipe.PIPE_G;
            case PIPE_I -> ReefPipe.PIPE_J;
            case PIPE_J -> ReefPipe.PIPE_I;
            case PIPE_K -> ReefPipe.PIPE_L;
            case PIPE_L -> ReefPipe.PIPE_K;
          };
      reefState.remove(partnerPipe, preferedScoringLevel);
      reefState.markScored(storedPipe, preferedScoringLevel);
      setPipeOveride(partnerPipe);
    }
  }

  public boolean isAligned(ReefPipe pipe) {
    if (pipeLevel.equals(ReefPipeLevel.RAISING) || pipeLevel.equals(ReefPipeLevel.BACK_AWAY)) {
      return false;
    }
    var robotPose = localization.getPose();
    var scoringPoseFieldRelative = getUsedScoringPose(pipe);
    var translationGood =
        (robotPose.getTranslation().getDistance(scoringPoseFieldRelative.getTranslation())
            <= TRANSLATION_GOOD_THRESHOLD.get());
    var rotationGood =
        MathUtil.isNear(
            scoringPoseFieldRelative.getRotation().getDegrees(),
            robotPose.getRotation().getDegrees(),
            ROTATION_GOOD_THRESHOLD.get(),
            -180.0,
            180.0);
    return translationGood && rotationGood;
  }


  public boolean needToMove(Pose2d goal) {
   var robotPose = localization.getPose();
    var translationBad =
        (robotPose.getTranslation().getDistance(goal.getTranslation())
            >NEED_TO_MOVE_TRANSLATION_THRESHOLD.get());
    var rotationBad =
        !MathUtil.isNear(
            goal.getRotation().getDegrees(),
            robotPose.getRotation().getDegrees(),
            NEED_TO_MOVE_ROTATION_THRESHOLD.get(),
            -180.0,
            180.0);
    return translationBad || rotationBad;
  }

  public boolean inRange(Pose2d goal) {
    var robotPose = localization.getPose();
     var translationGood =
         (robotPose.getTranslation().getDistance(goal.getTranslation())
             <=IN_RANGE_TRANSLATION_THRESHOLD.get());
     var rotationGood =
         MathUtil.isNear(
             goal.getRotation().getDegrees(),
             robotPose.getRotation().getDegrees(),
             IN_RANGE_ROTATION_THRESHOLD.get(),
             -180.0,
             180.0);
     return translationGood && rotationGood;
   }


  public void markScored(ReefPipe pipe) {
    reefState.markScored(pipe, preferedScoringLevel);
  }

  public void clearReefState() {
    reefState.clear();
  }

  public Pose2d getUsedScoringPose(ReefPipe pipe) {
    return getUsedScoringPose(pipe, robotScoringSide);
  }

  public Pose2d getUsedScoringPose(ReefPipe pipe, RobotScoringSide side) {
    return pipe.getPose(pipeLevel, side, localization.getPose());
  }

  /** Returns the best reef pipe for scoring, based on the robot's current state. */
  public ReefPipe getBestPipe() {
    if ((DriverStation.isAutonomous() || pipeSwitchActive) && reefPipeOverride.isPresent()) {
      return reefPipeOverride.orElseThrow();
    }
    var level = pipeLevel;
    var robotPose = localization.getPose();
    if (pipeLevel.equals(ReefPipeLevel.BACK_AWAY)) {
      return ALL_REEF_PIPES.stream()
          .min(
              Comparator.comparingDouble(
                  pipe ->
                      robotPose
                          .getTranslation()
                          .getDistance(
                              pipe.getPose(ReefPipeLevel.BACK_AWAY, robotScoringSide, robotPose)
                                  .getTranslation())))
          .orElseThrow();
    }
    if (pipeLevel.equals(ReefPipeLevel.RAISING)) {
      level = preferedScoringLevel;
    }
    return ALL_REEF_PIPES.stream()
        .min(alignmentCostUtil.getReefPipeComparator(level))
        .orElseThrow();
  }

  public PolarChassisSpeeds getPoseAlignmentChassisSpeeds(
      Pose2d targetPose,
      Pose2d currentPose,
      AutoConstraintOptions constraints,
      PolarChassisSpeeds currentSpeeds) {

if (FeatureFlags.AUTO_ALIGN_DEADBAND.getAsBoolean()) {
  if (aligned||inRange(targetPose)) {
    if (needToMove(targetPose)) {
      aligned = false;
    } else {
      aligned = true;
      return new PolarChassisSpeeds();
    }
  }
}

    // Calculate x and y velocities
    double distanceToGoalMeters =
        currentPose.getTranslation().getDistance(targetPose.getTranslation());

    var driveVelocityMagnitude = VELOCITY_CONTROLLER.calculate(distanceToGoalMeters);

    if (distanceToGoalMeters > TRANSLATION_GOOD_THRESHOLD.get()) {
      driveVelocityMagnitude += Math.copySign(FEED_FORWARD.get(), driveVelocityMagnitude);
    }

    MathUtil.clamp(driveVelocityMagnitude, -MAX_SPEED, MAX_SPEED);
    DogLog.log("AutoAlign/DistanceToGoal", distanceToGoalMeters);
    DogLog.log("AutoAlign/DriveVelocityMagnitude", driveVelocityMagnitude);

    var speeds =
        new PolarChassisSpeeds(
            driveVelocityMagnitude,
            MathHelpers.getDriveDirection(currentPose, targetPose),
            ROTATION_CONTROLLER.calculate(
                currentPose.getRotation().getRadians(), targetPose.getRotation().getRadians()));

    return speeds;
  }

  private final double lastMaxLinearAcceleration =
      new AutoConstraintOptions().maxLinearAcceleration();
}
