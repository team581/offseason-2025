package frc.robot.auto_align;

import com.team581.math.MathHelpers;
import com.team581.math.PolarChassisSpeeds;
import com.team581.math.PoseErrorTolerance;
import com.team581.trailblazer.constraints.AutoConstraintOptions;
import com.team581.util.state_machines.StateMachine;
import dev.doglog.DogLog;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.DoubleSubscriber;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.auto_align.tag_align.TagAlign;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.vision.VisionSubsystem;
import java.util.OptionalDouble;

public class AutoAlign extends StateMachine<AutoAlignState> {
  private static final AutoConstraintOptions CONSTRAINTS =
      new AutoConstraintOptions(3.0, 10.0, 2.0, 5.0);
  private static final Translation2d CENTER_OF_REEF_RED =
      new Translation2d(Units.inchesToMeters(514.13), Units.inchesToMeters(158.5));
  private static final Translation2d CENTER_OF_REEF_BLUE =
      new Translation2d(Units.inchesToMeters(176.746), Units.inchesToMeters(158.5));

  private static final DoubleSubscriber OBSTRUCTION_DISTANCE =
      DogLog.tunable("AutoAlign/ObstructionDistance", 0.75);

  private final PoseErrorTolerance positionTolerance = new PoseErrorTolerance(0.2, 4);

  public static Translation2d getAllianceCenterOfReef(boolean isRedAliance) {
    return isRedAliance ? CENTER_OF_REEF_RED : CENTER_OF_REEF_BLUE;
  }

  public static Translation2d getAllianceCenterOfReef(Pose2d robotPose) {
    return robotPose.getX() > 17.55 / 2 ? CENTER_OF_REEF_RED : CENTER_OF_REEF_BLUE;
  }

  public static boolean isCloseToReefSide(
      Pose2d robotPose, Pose2d nearestReefSide, double thresholdMeters) {
    return robotPose.getTranslation().getDistance(nearestReefSide.getTranslation())
        < thresholdMeters;
  }

  private final Debouncer isAlignedDebouncer = new Debouncer(0.0, DebounceType.kRising);
  private final VisionSubsystem vision;
  private final LocalizationSubsystem localization;
  private final TagAlign tagAlign;
  private final SwerveSubsystem swerve;

  private Pose2d robotPose = Pose2d.kZero;
  private ChassisSpeeds tagAlignSpeeds = new ChassisSpeeds();
  private ChassisSpeeds algaeAlignSpeeds = new ChassisSpeeds();
  private boolean isAligned = false;
  private boolean isNearRotation = false;
  private boolean isAlignedDebounced = false;
  private ReefPipe bestReefPipe = ReefPipe.PIPE_A;
  private Pose2d usedScoringPose = Pose2d.kZero;

  public AutoAlign(
      VisionSubsystem vision, LocalizationSubsystem localization, SwerveSubsystem swerve) {
    super(SubsystemPriority.AUTO_ALIGN, AutoAlignState.DEFAULT_STATE);

    this.tagAlign = new TagAlign(swerve, localization);
    this.vision = vision;
    this.localization = localization;
    this.swerve = swerve;
  }

  public void setAutoReefPipeOverride(ReefPipe override) {
    DogLog.log("Debug/SetPipeOverride", override);
    tagAlign.setPipeOveride(override);
  }

  public ReefSide getClosestReefSide() {
    return ReefSide.fromPipe(bestReefPipe);
  }

  @Override
  protected void collectInputs() {
    robotPose = localization.getPose();
    bestReefPipe = tagAlign.getBestPipe();
    usedScoringPose = tagAlign.getUsedScoringPose(bestReefPipe);
    isAligned = tagAlign.isAligned(bestReefPipe);
    isNearRotation = tagAlign.isNearRotationGoal(bestReefPipe);
    isAlignedDebounced = isAlignedDebouncer.calculate(isAligned);
    algaeAlignSpeeds =
        tagAlign.getPoseAlignmentChassisSpeeds(
            ReefSide.fromPipe(tagAlign.getClosestPipe())
                .getPose(robotPose),
            robotPose,
            CONSTRAINTS,
            new PolarChassisSpeeds(swerve.getFieldRelativeSpeeds()));
    tagAlignSpeeds =
        tagAlign.getPoseAlignmentChassisSpeeds(
            usedScoringPose,
            robotPose,
            CONSTRAINTS,
            new PolarChassisSpeeds(swerve.getFieldRelativeSpeeds()));
    var controllerValues = swerve.getControllerValues();
    tagAlign.setControllerValues(controllerValues.getX(), controllerValues.getY());
  }

  public boolean isNearRaisingPoint() {
    return positionTolerance.atPose(
        bestReefPipe.getPose(ReefPipeLevel.RAISING, robotPose), robotPose);
  }

  public boolean isNearRotationGoal() {
    return isNearRotation;
  }

  public int getL1ScoredCount() {
    return tagAlign.getL1ScoredCount(bestReefPipe);
  }

  public void setCoralL1Offset(OptionalDouble tx) {
    tagAlign.setCoralL1Offset(tx);
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();
    DogLog.log("AutoAlign/UsedScoringPose", usedScoringPose);
    DogLog.log("AutoAlign/IsAligned", isAligned);
    DogLog.log("AutoAlign/IsAlignedDebounced", isAlignedDebounced);
  }

  public ChassisSpeeds getTagAlignSpeeds() {
    DogLog.log("AutoAlign/TagAlignSpeeds", tagAlignSpeeds);
    return tagAlignSpeeds;
  }

  public ChassisSpeeds getAlgaeAlignSpeeds() {
    return algaeAlignSpeeds;
  }

  public ReefPipe getBestReefPipe() {
    return bestReefPipe;
  }

  public void markPipeScored() {
    tagAlign.markScored(bestReefPipe);
  }

  public void setScoringLevel(
      ReefPipeLevel level, ReefPipeLevel preferredLevel) {
    tagAlign.setLevel(level, preferredLevel);
  }

  public void clearReefState() {
    tagAlign.clearReefState();
  }

  public boolean isTagAlignedDebounced() {
    return isAlignedDebounced;
  }

  public Pose2d getUsedScoringPose() {
    return usedScoringPose;
  }

  public Pose2d getUsedScoringPose(ReefPipe pipe) {
    return tagAlign.getUsedScoringPose(pipe);
  }

  public Pose2d getUsedScoringPose(ReefPipe pipe, ReefPipeLevel level) {
    if (DriverStation.isAutonomous()) {
      tagAlign.setPipeOveride(pipe);
      setScoringLevel(level, level);
      return getUsedScoringPose(pipe);
    }
    return usedScoringPose;
  }

  public ReefAlignState getReefAlignState() {
    if (!vision.isAnyLeftScoringTagLimelightOnline()
        && !vision.isAnyRightScoringTagLimelightOnline()) {
      return ReefAlignState.ALL_CAMERAS_DEAD;
    }

    if (vision.getLeftBackTagResult().isPresent()
        || vision.getLeftFrontTagResult().isPresent()
        || vision.getRightTagResult().isPresent()
        || vision.getGamePieceTagResult().isPresent()) {
      if (isAligned) {
        return ReefAlignState.HAS_TAGS_IN_POSITION;
      }

      return ReefAlignState.HAS_TAGS_WRONG_POSITION;
    }

    if (isAligned) {
      return ReefAlignState.NO_TAGS_IN_POSITION;
    }
    return ReefAlignState.NO_TAGS_WRONG_POSITION;
  }
}
