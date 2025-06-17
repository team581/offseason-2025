package frc.robot.auto_align.tag_align;

import com.google.common.collect.ImmutableList;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import frc.robot.auto_align.AutoAlign;
import frc.robot.auto_align.ReefPipe;
import frc.robot.auto_align.ReefPipeLevel;
import frc.robot.auto_align.ReefState;
import frc.robot.auto_align.RobotScoringSide;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.MathHelpers;
import frc.robot.util.trailblazer.constraints.AutoConstraintCalculator;
import frc.robot.util.trailblazer.constraints.AutoConstraintOptions;
import frc.robot.util.trailblazer.trackers.pure_pursuit.PurePursuitUtils;

import java.util.Comparator;
import java.util.Optional;

public class TagAlign {
  private static final double SIDEWAYS_NOT_ALIGNED_FORWARD_SCALAR_ROTATION = 2.0;
  private static final double SIDEWAYS_NOT_ALIGNED_FORWARD_SCALAR_SIDEWAYS = 3.5;


  private static final int DRIVE_IN_ROTATION_THRESHOLD = 5;

  private static final double SIDEWAYS_ALIGNED_THRESHOLD = 0.05;
  private  final AutoConstraintOptions CONSTRAINT_OPTIONS =
      new AutoConstraintOptions(3.0,3,4.5,10);

  private static final ImmutableList<ReefPipe> ALL_REEF_PIPES = ImmutableList.copyOf(ReefPipe.values());

  private static final PIDController TRANSLATION_PID = new PIDController(5.0, 0.0, 0.0);
  private static final PIDController ROTATION_PID = new PIDController(7.0, 0.0, 0.0);
  private static final DoubleSubscriber TRANSLATION_GOOD_THRESHOLD = DogLog.tunable("AutoAlign/IsAlignedTranslation",
      0.03);
  private static final DoubleSubscriber ROTATION_GOOD_THRESHOLD = DogLog.tunable("AutoAlign/IsAlignedRotation", 3.0);

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

  private boolean pipeSwitchActive = false;

  public TagAlign(SwerveSubsystem swerve, LocalizationSubsystem localization) {
    this.localization = localization;
    alignmentCostUtil = new AlignmentCostUtil(localization, swerve, reefState, robotScoringSide);
    ROTATION_PID.enableContinuousInput(-Math.PI, Math.PI);
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
      ReefPipe partnerPipe = switch (storedPipe) {
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

      setPipeOveride(partnerPipe);
    }
  }

  public boolean isAligned(ReefPipe pipe) {
    if (pipeLevel.equals(ReefPipeLevel.RAISING) || pipeLevel.equals(ReefPipeLevel.BACK_AWAY)) {
      return false;
    }
    var robotPose = localization.getPose();
    var scoringPoseFieldRelative = getUsedScoringPose(pipe);
    var translationGood = (robotPose.getTranslation()
        .getDistance(scoringPoseFieldRelative.getTranslation()) <= TRANSLATION_GOOD_THRESHOLD.get());
    var rotationGood = MathUtil.isNear(
        scoringPoseFieldRelative.getRotation().getDegrees(),
        robotPose.getRotation().getDegrees(),
        ROTATION_GOOD_THRESHOLD.get(),
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
    return pipe.getPose(pipeLevel, robotScoringSide, localization.getPose());
  }

  /**
   * Returns the best reef pipe for scoring, based on the robot's current state.
   */
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
                  pipe -> robotPose
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

  public ChassisSpeeds getPoseAlignmentChassisSpeeds(Pose2d usedScoringPose) {
    var robotPose = localization.getPose();

    var scoringTranslationRobotRelative = usedScoringPose
        .getTranslation()
        .minus(robotPose.getTranslation())
        .rotateBy(Rotation2d.fromDegrees(360 - usedScoringPose.getRotation().getDegrees()));

    var goalTranslationWithP = new Translation2d(
        TRANSLATION_PID.calculate(scoringTranslationRobotRelative.getX()),
        TRANSLATION_PID.calculate(scoringTranslationRobotRelative.getY()));

    if (Math.abs(scoringTranslationRobotRelative.getX()) > SIDEWAYS_ALIGNED_THRESHOLD
        || !MathUtil.isNear(usedScoringPose.getRotation().getDegrees(), robotPose.getRotation().getDegrees(),
            DRIVE_IN_ROTATION_THRESHOLD)) {
      goalTranslationWithP = new Translation2d(
          TRANSLATION_PID.calculate(scoringTranslationRobotRelative.getX()),
          TRANSLATION_PID.calculate(scoringTranslationRobotRelative.getY() / MathUtil.clamp((Math.abs(scoringTranslationRobotRelative.getY()* SIDEWAYS_NOT_ALIGNED_FORWARD_SCALAR_SIDEWAYS)+(Math.abs(usedScoringPose.getRotation().minus(robotPose.getRotation()).getRadians())* SIDEWAYS_NOT_ALIGNED_FORWARD_SCALAR_ROTATION)) , 1.0, Double.MAX_VALUE)));
    }
    var goalTranslation = goalTranslationWithP.rotateBy(usedScoringPose.getRotation());

    var goalSpeeds = new ChassisSpeeds(-goalTranslation.getX(), -goalTranslation.getY(),
        ROTATION_PID.calculate(robotPose.getRotation().getRadians(),
            usedScoringPose.getRotation().getRadians()));
    var constrainedSpeeds = AutoConstraintCalculator.constrainLinearVelocity(new ChassisSpeeds(goalSpeeds.vxMetersPerSecond, goalSpeeds.vyMetersPerSecond, MathUtil.clamp(goalSpeeds.omegaRadiansPerSecond, -CONSTRAINT_OPTIONS.maxAngularVelocity(), CONSTRAINT_OPTIONS.maxAngularVelocity())), CONSTRAINT_OPTIONS);
    DogLog.log("AutoAlign/GoalSpeeds", goalSpeeds);
    DogLog.log("AutoAlign/ConstrainedSpeeds", constrainedSpeeds);

    return constrainedSpeeds;
  }
}
