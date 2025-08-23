package frc.robot.robot_manager;

import com.team581.math.MathHelpers;
import com.team581.util.state_machines.StateMachine;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.arm.ArmState;
import frc.robot.arm.ArmSubsystem;
import frc.robot.auto_align.AutoAlign;
import frc.robot.auto_align.ReefPipeLevel;
import frc.robot.auto_align.ReefSide;
import frc.robot.claw.ClawState;
import frc.robot.claw.ClawSubsystem;
import frc.robot.climber.ClimberState;
import frc.robot.climber.ClimberSubsystem;
import frc.robot.config.FeatureFlags;
import frc.robot.controller.RumbleControllerSubsystem;
import frc.robot.elevator.ElevatorState;
import frc.robot.elevator.ElevatorSubsystem;
import frc.robot.imu.ImuSubsystem;
import frc.robot.intake_assist.IntakeAssistUtil;
import frc.robot.lights.LightsState;
import frc.robot.lights.LightsSubsystem;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.robot_manager.collision_avoidance.CollisionAvoidance;
import frc.robot.robot_manager.collision_avoidance.ObstructionKind;
import frc.robot.robot_manager.ground_manager.GroundManager;
import frc.robot.robot_manager.ground_manager.GroundState;
import frc.robot.swerve.SnapUtil;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;
import frc.robot.vision.VisionState;
import frc.robot.vision.VisionSubsystem;
import frc.robot.vision.game_piece_detection.CoralMap;
import java.util.Optional;

public class RobotManager extends StateMachine<RobotState> {
  public final LocalizationSubsystem localization;

  public final VisionSubsystem vision;
  public final ImuSubsystem imu;

  public final CoralMap coralMap;
  public final SwerveSubsystem swerve;
  public final ClawSubsystem claw;
  public final ArmSubsystem arm;
  public final ElevatorSubsystem elevator;
  public final ClimberSubsystem climber;
  public final RumbleControllerSubsystem rumbleController;

  public final GroundManager groundManager;

  private final LightsSubsystem lights;

  public final AutoAlign autoAlign;

  public RobotManager(
      GroundManager groundManager,
      ClawSubsystem claw,
      ArmSubsystem arm,
      ElevatorSubsystem elevator,
      VisionSubsystem vision,
      ImuSubsystem imu,
      SwerveSubsystem swerve,
      LocalizationSubsystem localization,
      CoralMap coralMap,
      LightsSubsystem lights,
      AutoAlign autoAlign,
      ClimberSubsystem climber,
      RumbleControllerSubsystem rumbleController) {
    super(SubsystemPriority.ROBOT_MANAGER, RobotState.STARTING_POSITION);
    this.groundManager = groundManager;
    this.arm = arm;
    this.claw = claw;
    this.elevator = elevator;
    this.vision = vision;
    this.imu = imu;
    this.swerve = swerve;
    this.localization = localization;
    this.coralMap = coralMap;
    this.lights = lights;
    this.climber = climber;
    this.autoAlign = autoAlign;
    this.rumbleController = rumbleController;

    var stateCount = RobotState.values().length;

    DogLog.log("RobotManager/StateCount", stateCount);
  }

  private double reefSnapAngle = 0.0;
  private ReefSide nearestReefSide = ReefSide.SIDE_GH;
  private ReefPipeLevel scoringLevel = ReefPipeLevel.L4;
  private Pose2d robotPose;
  private ObstructionKind shouldLoopAroundToScoreObstruction = ObstructionKind.NONE;
  private Optional<RobotState> afterIntakingCoralState = Optional.empty();
  private boolean scoringAlignActive = false;
  private boolean canSkipCollisionAvoidanceForReefAlgae = false;

  private boolean intakeAssistActive = false;

  @Override
  protected RobotState getNextState(RobotState currentState) {
    if (afterIntakingCoralState.isPresent() && groundManager.hasCoral()) {
      return afterIntakingCoralState.orElseThrow();
    }

    return switch (currentState) {
      case CLAW_EMPTY,
          CLAW_ALGAE,
          CLAW_CORAL,
          STARTING_POSITION_CORAL,
          STARTING_POSITION,
          CLAW_ALGAE_STOW_INWARD,
          LOW_STOW,
          ALGAE_INTAKE_FLOOR,
          ALGAE_PROCESSOR_WAITING,
          ALGAE_NET_WAITING,
          CLIMBING_2_HANGING,
          CLIMBER_STOP,
          UNJAM,
          SPIN_TO_WIN,
          CORAL_INTAKE_LOLLIPOP_APPROACH,
          ALGAE_FLING_WAIT,
          FORCED_HANDOFF,
          FORCED_LOWSTOW ->
          currentState;
      case CORAL_L2_PLACE,
          CORAL_L3_PLACE,
          CORAL_L4_PLACE -> {
        if (((FeatureFlags.AUTO_ALIGN_AUTO_SCORE.getAsBoolean() && scoringAlignActive)
                || DriverStation.isAutonomous())
            && ((arm.atGoal() && elevator.atGoal()) || timeout(0.15))) {
          autoAlign.markPipeScored();
          yield currentState.getPlaceToReleaseState();
        }
        yield currentState;
      }
      case CORAL_L1_LINEUP,
          CORAL_L2_LINEUP,
          CORAL_L3_LINEUP,
          CORAL_L4_LINEUP-> {
        if (((FeatureFlags.AUTO_ALIGN_AUTO_SCORE.getAsBoolean() && scoringAlignActive)
                || DriverStation.isAutonomous())
            && (autoAlign.isTagAlignedDebounced() || (DriverStation.isAutonomous() && timeout(3.0)))
            && arm.atGoal()
            && elevator.atGoal()) {
          yield currentState.getLineupToPlaceState();
        }
        yield currentState;
      }
      case REHOME_ELEVATOR ->
          elevator.getState() == ElevatorState.STOWED ? RobotState.CLAW_EMPTY : currentState;
      case PREPARE_SPIN_TO_WIN ->
          elevator.atGoal() && arm.atGoal() ? RobotState.SPIN_TO_WIN : currentState;

      case ALGAE_FLING_PREPARE -> arm.atGoal() ? RobotState.ALGAE_FLING_RELEASE : currentState;
      case ALGAE_FLING_RELEASE -> !claw.getHasGP() ? RobotState.CLAW_EMPTY : currentState;

      case CORAL_L1_PREPARE_HANDOFF,
          CORAL_L2_PREPARE_HANDOFF,
          CORAL_L3_PREPARE_HANDOFF,
          CORAL_L4_PREPARE_HANDOFF ->
          elevator.atGoal()
                  && arm.atGoal()
                  && groundManager.deploy.atGoal()
                  && groundManager.getState().equals(GroundState.HANDOFF_WAIT)
                  && groundManager.intake.getHasGP()
              ? currentState.getHandoffPrepareToReleaseState()
              : currentState;

      case CORAL_L1_RELEASE_HANDOFF,
          CORAL_L2_RELEASE_HANDOFF,
          CORAL_L3_RELEASE_HANDOFF,
          CORAL_L4_RELEASE_HANDOFF ->
          claw.getHasGP() ? currentState.getHandoffReleaseToAfterRelease() : currentState;
      case CORAL_L1_AFTER_RELEASE_HANDOFF,
          CORAL_L2_AFTER_RELEASE_HANDOFF,
          CORAL_L3_AFTER_RELEASE_HANDOFF,
          CORAL_L4_AFTER_RELEASE_HANDOFF ->
          elevator.atGoal() && arm.atGoal()
              ? currentState.getHandoffAfterReleaseToApproachState(robotScoringSide)
              : currentState;

      // Approach
      case CORAL_L1_RIGHT_APPROACH ->
          elevator.nearGoal() && arm.nearGoal()
              ? currentState.getRightApproachToLineupState()
              : currentState;

      case CORAL_L2_LEFT_APPROACH -> {
        yield elevator.nearGoal(ElevatorState.CORAL_SCORE_LINEUP_L2)
                && arm.nearGoal(ArmState.CORAL_SCORE_LEFT_LINEUP_L2)
                && (!FeatureFlags.APPROACH_TAG_CHECK.getAsBoolean() || vision.seeingTag())
                && autoAlign.isNearRotationGoal()
            ? currentState.getLeftApproachToLineupState()
            : currentState;
      }
      case CORAL_L3_LEFT_APPROACH -> {
        yield elevator.nearGoal(ElevatorState.CORAL_SCORE_LINEUP_L3)
                && arm.nearGoal(ArmState.CORAL_SCORE_LEFT_LINEUP_L3)
                && (!FeatureFlags.APPROACH_TAG_CHECK.getAsBoolean() || vision.seeingTag())
                && autoAlign.isNearRotationGoal()
            ? currentState.getLeftApproachToLineupState()
            : currentState;
      }
      case CORAL_L4_LEFT_APPROACH -> {
        yield elevator.nearGoal(ElevatorState.L4_SCORE_LINEUP)
                && arm.nearGoal(ArmState.CORAL_SCORE_LEFT_LINEUP_L4)
                && (!FeatureFlags.APPROACH_TAG_CHECK.getAsBoolean() || vision.seeingTag())
                && autoAlign.isNearRotationGoal()
            ? currentState.getLeftApproachToLineupState()
            : currentState;
      }

      case CORAL_L2_RIGHT_APPROACH -> {
        yield elevator.nearGoal(ElevatorState.CORAL_SCORE_LINEUP_L2)
                && arm.nearGoal(ArmState.CORAL_SCORE_RIGHT_LINEUP_L2)
                && (!FeatureFlags.APPROACH_TAG_CHECK.getAsBoolean() || vision.seeingTag())
                && autoAlign.isNearRotationGoal()
            ? currentState.getRightApproachToLineupState()
            : currentState;
      }
      case CORAL_L3_RIGHT_APPROACH -> {
        yield elevator.nearGoal(ElevatorState.CORAL_SCORE_LINEUP_L3)
                && arm.nearGoal(ArmState.CORAL_SCORE_RIGHT_LINEUP_L3)
                && (!FeatureFlags.APPROACH_TAG_CHECK.getAsBoolean() || vision.seeingTag())
                && autoAlign.isNearRotationGoal()
            ? currentState.getRightApproachToLineupState()
            : currentState;
      }
      case CORAL_L4_RIGHT_APPROACH -> {
        yield elevator.nearGoal(ElevatorState.L4_SCORE_LINEUP)
                && arm.nearGoal(ArmState.CORAL_SCORE_RIGHT_LINEUP_L4)
                && (!FeatureFlags.APPROACH_TAG_CHECK.getAsBoolean() || vision.seeingTag())
                && autoAlign.isNearRotationGoal()
            ? currentState.getRightApproachToLineupState()
            : currentState;
      }

      case CORAL_L2_RELEASE,
          CORAL_L3_RELEASE,
          CORAL_L4_RELEASE,
          CORAL_L1_RELEASE -> {
        if (DriverStation.isTeleop()) {
          // In teleop, we go to CLAW_EMPTY when you drive away or if we know the score succeeded
          if (drivingAwayFromReef()) {
            yield RobotState.CLAW_EMPTY;
          }
        }

        yield currentState;
      }

      // Algae scoring
      case ALGAE_PROCESSOR_RELEASE, ALGAE_NET_LEFT_RELEASE, ALGAE_NET_RIGHT_RELEASE -> {
        if (FeatureFlags.AUTO_STOW_ALGAE.getAsBoolean()) {
          yield timeout(0.5) || !claw.getHasGP() ? RobotState.CLAW_EMPTY : currentState;
        }

        yield currentState;
      }

      case ALGAE_OUTTAKE -> timeout(0.5) || !claw.getHasGP() ? RobotState.CLAW_EMPTY : currentState;

      // Intaking

      case ALGAE_INTAKE_L2_APPROACH,
          ALGAE_INTAKE_L3_APPROACH ->
          arm.nearGoal() && elevator.nearGoal()
              ? currentState.getAlgaeApproachToIntakeState()
              : currentState;

      case ALGAE_INTAKE_L2,
          ALGAE_INTAKE_L3 -> {
        if (claw.getHasGP()) {
          rumbleController.rumbleRequest();
          yield currentState.getAlgaeIntakeToHoldingState();
        }

        yield currentState;
      }

      case ALGAE_INTAKE_L2_HOLDING,
          ALGAE_INTAKE_L3_HOLDING -> {
        if (cameraOnlineAndFarEnoughFromReef()) {
          // yield DriverStation.isAutonomous() ? RobotState.CLAW_ALGAE_STOW_INWARD :
          // RobotState.CLAW_ALGAE;
          // TODO: implement stow inward
          yield RobotState.CLAW_ALGAE;
        }

        yield currentState;
      }

      case CORAL_INTAKE_LOLLIPOP_GRAB ->
          claw.getHasGP() ? RobotState.CORAL_INTAKE_LOLLIPOP_PUSH : currentState;
      case CORAL_INTAKE_LOLLIPOP_PUSH -> timeout(0.6) ? RobotState.CLAW_CORAL : currentState;

      case CLIMBING_1_LINEUP ->
          climber.holdingCage() ? RobotState.CLIMBING_2_HANGING : currentState;
    };
  }

  @Override
  protected void afterTransition(RobotState newState) {
    switch (newState) {
      case CLAW_EMPTY -> {
        claw.setState(ClawState.IDLE_NO_GP);
        moveSuperstructure(ElevatorState.PRE_CORAL_HANDOFF, ArmState.CORAL_HANDOFF);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.IDLE_EMPTY);
        climber.setState(ClimberState.STOPPED);
      }
      case CLAW_ALGAE -> {
        claw.setState(ClawState.IDLE_W_ALGAE);
        moveSuperstructure(ElevatorState.STOWED, ArmState.HOLDING_UPRIGHT);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.HOLDING_ALGAE);
        climber.setState(ClimberState.STOPPED);
      }
      case CLAW_CORAL -> {
        claw.setState(ClawState.IDLE_W_CORAL);
        moveSuperstructure(ElevatorState.PRE_CORAL_HANDOFF, ArmState.CORAL_HANDOFF);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.HOLDING_CORAL);
        climber.setState(ClimberState.STOPPED);
      }
      case STARTING_POSITION, LOW_STOW -> {
        claw.setState(ClawState.IDLE_NO_GP);
        moveSuperstructure(ElevatorState.STOWED, ArmState.HOLDING_UPRIGHT);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.IDLE_EMPTY);
        climber.setState(ClimberState.STOPPED);
      }
      case FORCED_LOWSTOW -> {
        claw.setState(ClawState.IDLE_NO_GP);
        moveSuperstructure(ElevatorState.STOWED, ArmState.HOLDING_UPRIGHT, true);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.OTHER);
        climber.setState(ClimberState.STOPPED);
      }
      case FORCED_HANDOFF -> {
        claw.setState(ClawState.IDLE_NO_GP);
        moveSuperstructure(ElevatorState.PRE_CORAL_HANDOFF, ArmState.CORAL_HANDOFF, true);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.OTHER);
        climber.setState(ClimberState.STOPPED);
      }
      case STARTING_POSITION_CORAL -> {
        claw.setState(ClawState.IDLE_W_CORAL);
        moveSuperstructure(ElevatorState.STOWED, ArmState.HOLDING_UPRIGHT);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.HOLDING_CORAL);
        climber.setState(ClimberState.STOPPED);
      }
      case ALGAE_INTAKE_FLOOR -> {
        claw.setState(ClawState.INTAKING_ALGAE);
        moveSuperstructure(ElevatorState.ALGAE_INTAKE_GROUND, ArmState.ALGAE_INTAKE_FLOOR);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.INTAKING_ALGAE);
        climber.setState(ClimberState.STOPPED);
      }
      case ALGAE_INTAKE_L2, ALGAE_INTAKE_L2_HOLDING -> {
        claw.setState(ClawState.INTAKING_ALGAE);
        moveSuperstructure(
            ElevatorState.ALGAE_INTAKE_L2,
            ArmState.ALGAE_INTAKE_LEFT_L2,
            canSkipCollisionAvoidanceForReefAlgae);
        swerve.setSnapToAngle(reefSnapAngle);
        vision.setState(VisionState.CLOSEST_REEF_TAG);
        lights.setState(LightsState.INTAKING_ALGAE);
        climber.setState(ClimberState.STOPPED);
      }
      case ALGAE_INTAKE_L2_APPROACH -> {
        claw.setState(ClawState.INTAKING_ALGAE);
        moveSuperstructure(
            ElevatorState.ALGAE_INTAKE_L2,
           ArmState.ALGAE_INTAKE_RIGHT_L2,
            canSkipCollisionAvoidanceForReefAlgae);
        swerve.setSnapToAngle(reefSnapAngle);
        vision.setState(VisionState.CLOSEST_REEF_TAG);
        lights.setState(LightsState.INTAKING_ALGAE);

        climber.setState(ClimberState.STOPPED);
      }
      case ALGAE_INTAKE_L2, ALGAE_INTAKE_L2_HOLDING -> {
        claw.setState(ClawState.INTAKING_ALGAE);
        moveSuperstructure(
            ElevatorState.ALGAE_INTAKE_L2,
            ArmState.ALGAE_INTAKE_RIGHT_L2,
            canSkipCollisionAvoidanceForReefAlgae);
        swerve.setSnapToAngle(reefSnapAngle);
        vision.setState(VisionState.CLOSEST_REEF_TAG);
        lights.setState(LightsState.INTAKING_ALGAE);
        climber.setState(ClimberState.STOPPED);
      }
      case ALGAE_INTAKE_L3_APPROACH -> {
        claw.setState(ClawState.INTAKING_ALGAE);
        moveSuperstructure(
            ElevatorState.ALGAE_INTAKE_L3,
            ArmState.ALGAE_INTAKE_LEFT_L3,
            canSkipCollisionAvoidanceForReefAlgae);
        swerve.setSnapToAngle(reefSnapAngle);
        vision.setState(VisionState.CLOSEST_REEF_TAG);
        lights.setState(LightsState.INTAKING_ALGAE);

        climber.setState(ClimberState.STOPPED);
      }
      case ALGAE_INTAKE_L3, ALGAE_INTAKE_L3_HOLDING -> {
        claw.setState(ClawState.INTAKING_ALGAE);
        moveSuperstructure(
            ElevatorState.ALGAE_INTAKE_L3,
            ArmState.ALGAE_INTAKE_LEFT_L3,
            canSkipCollisionAvoidanceForReefAlgae);
        swerve.setSnapToAngle(reefSnapAngle);
        vision.setState(VisionState.CLOSEST_REEF_TAG);
        lights.setState(LightsState.INTAKING_ALGAE);
        climber.setState(ClimberState.STOPPED);
      }
      case ALGAE_INTAKE_L3_APPROACH -> {
        claw.setState(ClawState.INTAKING_ALGAE);
        moveSuperstructure(
            ElevatorState.ALGAE_INTAKE_L3,
            ArmState.ALGAE_INTAKE_RIGHT_L3,
            canSkipCollisionAvoidanceForReefAlgae);
        swerve.setSnapToAngle(reefSnapAngle);
        vision.setState(VisionState.CLOSEST_REEF_TAG);
        lights.setState(LightsState.INTAKING_ALGAE);

        climber.setState(ClimberState.STOPPED);
      }
      case CORAL_INTAKE_LOLLIPOP_APPROACH -> {
        claw.setState(ClawState.IDLE_NO_GP);
        moveSuperstructure(
            ElevatorState.LOLLIPOP_CORAL_INTAKE_INTAKE, ArmState.LOLLIPOP_CORAL_INTAKE_INTAKE);
        swerve.normalDriveRequest();
        vision.setState(VisionState.ALGAE_DETECTION);
        lights.setState(LightsState.LOLLIPOP_NO_ALGAE);
        climber.setState(ClimberState.STOPPED);
      }
      case CORAL_INTAKE_LOLLIPOP_GRAB -> {
        claw.setState(ClawState.LOLLIPOP_CORAL_INTAKE);
        moveSuperstructure(
            ElevatorState.LOLLIPOP_CORAL_INTAKE_INTAKE, ArmState.LOLLIPOP_CORAL_INTAKE_INTAKE);
        swerve.normalDriveRequest();
        vision.setState(VisionState.ALGAE_DETECTION);
        lights.setState(LightsState.LOLLIPOP_NO_ALGAE);
        climber.setState(ClimberState.STOPPED);
      }
      case CORAL_INTAKE_LOLLIPOP_PUSH -> {
        claw.setState(ClawState.IDLE_W_CORAL);
        // You are only ever doing this from the grab state, so this is fine
        moveSuperstructure(
            ElevatorState.LOLLIPOP_CORAL_INTAKE_PUSH, ArmState.LOLLIPOP_CORAL_INTAKE_PUSH, true);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.LOLLIPOP_NO_ALGAE);
        climber.setState(ClimberState.STOPPED);
      }
      case ALGAE_NET_WAITING -> {
        claw.setState(ClawState.IDLE_W_ALGAE);
        moveSuperstructure(ElevatorState.ALGAE_NET, ArmState.ALGAE_NET_RIGHT);
        swerve.snapsDriveRequest(SnapUtil.getNetScoringAngle(RobotScoringSide.RIGHT, robotPose));
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.HOLDING_ALGAE);
        climber.setState(ClimberState.STOPPED);
      }
      case ALGAE_NET_RELEASE -> {
        claw.setState(ClawState.SCORE_ALGAE_NET);
        moveSuperstructure(ElevatorState.ALGAE_NET, ArmState.ALGAE_NET_LEFT);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.SCORING_ALGAE);
        climber.setState(ClimberState.STOPPED);
      }
      case ALGAE_PROCESSOR_WAITING -> {
        claw.setState(ClawState.IDLE_W_ALGAE);
        moveSuperstructure(ElevatorState.PROCESSOR, ArmState.ALGAE_PROCESSOR);
        swerve.snapsDriveRequest(SnapUtil.getProcessorAngle());
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.HOLDING_ALGAE);
        climber.setState(ClimberState.STOPPED);
      }
      case ALGAE_PROCESSOR_RELEASE -> {
        claw.setState(ClawState.SCORE_ALGAE_PROCESSOR);
        moveSuperstructure(ElevatorState.PROCESSOR, ArmState.ALGAE_PROCESSOR);
        swerve.snapsDriveRequest(SnapUtil.getProcessorAngle());
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.SCORING_ALGAE);
        climber.setState(ClimberState.STOPPED);
      }
      // Handoff states
      case CORAL_L1_PREPARE_HANDOFF,
          CORAL_L2_PREPARE_HANDOFF,
          CORAL_L3_PREPARE_HANDOFF,
          CORAL_L4_PREPARE_HANDOFF -> {
        claw.setState(ClawState.CORAL_HANDOFF);
        afterIntakingCoralState = Optional.empty();
        groundManager.intakeThenHandoffRequest();
        moveSuperstructure(ElevatorState.PRE_CORAL_HANDOFF, ArmState.CORAL_HANDOFF);
        swerve.normalDriveRequest();
        vision.setState(VisionState.HANDOFF);
        lights.setState(LightsState.CORAL_HANDOFF);
        climber.setState(ClimberState.STOPPED);
      }
      case CORAL_L1_RELEASE_HANDOFF,
          CORAL_L2_RELEASE_HANDOFF,
          CORAL_L3_RELEASE_HANDOFF,
          CORAL_L4_RELEASE_HANDOFF -> {
        claw.setState(ClawState.CORAL_HANDOFF);
        groundManager.handoffReleaseRequest();
        moveSuperstructure(ElevatorState.CORAL_HANDOFF, ArmState.CORAL_HANDOFF);
        swerve.normalDriveRequest();
        vision.setState(VisionState.HANDOFF);
        lights.setState(LightsState.CORAL_HANDOFF);
        climber.setState(ClimberState.STOPPED);
      }
      case CORAL_L1_AFTER_RELEASE_HANDOFF,
          CORAL_L2_AFTER_RELEASE_HANDOFF,
          CORAL_L3_AFTER_RELEASE_HANDOFF,
          CORAL_L4_AFTER_RELEASE_HANDOFF -> {
        claw.setState(ClawState.CORAL_HANDOFF);
        groundManager.handoffReleaseRequest();
        moveSuperstructure(ElevatorState.PRE_CORAL_HANDOFF, ArmState.CORAL_HANDOFF);
        swerve.normalDriveRequest();
        vision.setState(VisionState.HANDOFF);
        lights.setState(LightsState.CORAL_HANDOFF);
        climber.setState(ClimberState.STOPPED);
      }

      // L1
      case CORAL_L1_APPROACH -> {
        claw.setState(ClawState.IDLE_W_CORAL);
        moveSuperstructure(
            ElevatorState.CORAL_SCORE_RIGHT_LINEUP_L1, ArmState.CORAL_SCORE_RIGHT_LINEUP_L1);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(getLightStateForScoring());
        climber.setState(ClimberState.STOPPED);
      }
      case CORAL_L1_LINEUP -> {
        claw.setState(ClawState.IDLE_W_CORAL);
        moveSuperstructure(
            ElevatorState.CORAL_SCORE_RIGHT_LINEUP_L1, ArmState.CORAL_SCORE_RIGHT_LINEUP_L1);
        swerve.normalDriveRequest();
        vision.setState(VisionState.CLOSEST_REEF_TAG);
        lights.setState(getLightStateForScoring());
        climber.setState(ClimberState.STOPPED);
      }
      case CORAL_L1_RELEASE -> {
        claw.setState(ClawState.SCORE_CORAL);
        moveSuperstructure(
            ElevatorState.CORAL_SCORE_RIGHT_RELEASE_L1,
            ArmState.CORAL_SCORE_RIGHT_RELEASE_L1,
            true);
        swerve.normalDriveRequest();
        vision.setState(VisionState.CLOSEST_REEF_TAG);
        lights.setState(LightsState.SCORING_CORAL);
        climber.setState(ClimberState.STOPPED);
      }
      // L2
      case CORAL_L2_APPROACH -> {
        claw.setState(ClawState.IDLE_W_CORAL);
        moveSuperstructure(
            ElevatorState.CORAL_SCORE_LINEUP_L2, ArmState.CORAL_SCORE_LEFT_LINEUP_L2);
        swerve.setSnapToAngle(reefSnapAngle);
        vision.setState(VisionState.TAGS);
        lights.setState(getLightStateForScoring());

        climber.setState(ClimberState.STOPPED);
      }
      case CORAL_L2_LINEUP -> {
        claw.setState(ClawState.IDLE_W_CORAL);
        moveSuperstructure(
            ElevatorState.CORAL_SCORE_LINEUP_L2, ArmState.CORAL_SCORE_LEFT_LINEUP_L2);
        swerve.setSnapToAngle(reefSnapAngle);
        vision.setState(VisionState.CLOSEST_REEF_TAG);

        lights.setState(getLightStateForScoring());
        climber.setState(ClimberState.STOPPED);
      }
      case CORAL_L2_PLACE -> {
        claw.setState(ClawState.IDLE_NO_GP);
        moveSuperstructure(
            ElevatorState.CORAL_SCORE_RELEASE_L2, ArmState.CORAL_SCORE_LEFT_RELEASE_L2, true);
        vision.setState(VisionState.CLOSEST_REEF_TAG);
        lights.setState(getLightStateForScoring());
        climber.setState(ClimberState.STOPPED);
      }
      case CORAL_L2_RELEASE -> {
        claw.setState(ClawState.SCORE_CORAL);
        moveSuperstructure(
            ElevatorState.CORAL_SCORE_RELEASE_L2, ArmState.CORAL_SCORE_LEFT_RELEASE_L2, true);
        vision.setState(VisionState.CLOSEST_REEF_TAG);
        lights.setState(LightsState.SCORING_CORAL);

        climber.setState(ClimberState.STOPPED);
      }
      // L3
      case CORAL_L3_APPROACH -> {
        claw.setState(ClawState.IDLE_W_CORAL);
        moveSuperstructure(
            ElevatorState.CORAL_SCORE_LINEUP_L3, ArmState.CORAL_SCORE_LEFT_LINEUP_L3);
        swerve.setSnapToAngle(reefSnapAngle);
        vision.setState(VisionState.TAGS);
        lights.setState(getLightStateForScoring());
        climber.setState(ClimberState.STOPPED);
      }
      case CORAL_L3_LINEUP -> {
        claw.setState(ClawState.IDLE_W_CORAL);
        moveSuperstructure(
            ElevatorState.CORAL_SCORE_LINEUP_L3, ArmState.CORAL_SCORE_LEFT_LINEUP_L3);
        swerve.setSnapToAngle(reefSnapAngle);
        vision.setState(VisionState.CLOSEST_REEF_TAG);
        lights.setState(getLightStateForScoring());

        climber.setState(ClimberState.STOPPED);
      }
      case CORAL_L3_PLACE -> {
        claw.setState(ClawState.IDLE_NO_GP);
        moveSuperstructure(
            ElevatorState.CORAL_SCORE_RELEASE_L3, ArmState.CORAL_SCORE_LEFT_RELEASE_L3, true);
        vision.setState(VisionState.CLOSEST_REEF_TAG);
        lights.setState(getLightStateForScoring());
        climber.setState(ClimberState.STOPPED);
      }
      case CORAL_L3_RELEASE -> {
        claw.setState(ClawState.SCORE_CORAL);
        moveSuperstructure(
            ElevatorState.CORAL_SCORE_RELEASE_L3, ArmState.CORAL_SCORE_LEFT_RELEASE_L3, true);
        vision.setState(VisionState.CLOSEST_REEF_TAG);
        lights.setState(LightsState.SCORING_CORAL);

        climber.setState(ClimberState.STOPPED);
      }
      // L4
      case CORAL_L4_APPROACH -> {
        claw.setState(ClawState.IDLE_W_CORAL);
        moveSuperstructure(
            ElevatorState.L4_SCORE_LINEUP, ArmState.CORAL_SCORE_LEFT_LINEUP_L4);
        swerve.setSnapToAngle(reefSnapAngle);
        vision.setState(VisionState.TAGS);
        lights.setState(getLightStateForScoring());

        climber.setState(ClimberState.STOPPED);
      }
      case CORAL_L4_LINEUP -> {
        claw.setState(ClawState.IDLE_W_CORAL);
        moveSuperstructure(
            ElevatorState.L4_SCORE_LINEUP, ArmState.CORAL_SCORE_LEFT_LINEUP_L4);
        swerve.setSnapToAngle(reefSnapAngle);
        vision.setState(VisionState.CLOSEST_REEF_TAG);
        lights.setState(getLightStateForScoring());

        climber.setState(ClimberState.STOPPED);
      }
      case CORAL_L4_PLACE -> {
        claw.setState(ClawState.IDLE_NO_GP);
        moveSuperstructure(
            ElevatorState.L4_SCORE_RELEASE, ArmState.CORAL_SCORE_LEFT_RELEASE_L4, true);
        vision.setState(VisionState.CLOSEST_REEF_TAG);
        lights.setState(getLightStateForScoring());
        climber.setState(ClimberState.STOPPED);
      }
      case CORAL_L4_RELEASE -> {
        claw.setState(ClawState.SCORE_CORAL);
        moveSuperstructure(
            ElevatorState.L4_SCORE_RELEASE, ArmState.CORAL_SCORE_LEFT_RELEASE_L4, true);
        vision.setState(VisionState.CLOSEST_REEF_TAG);
        lights.setState(LightsState.SCORING_CORAL);

        climber.setState(ClimberState.STOPPED);
      }
      // Climb
      case CLIMBING_1_LINEUP -> {
        claw.setState(ClawState.IDLE_NO_GP);
        groundManager.climbRequest();
        moveSuperstructure(ElevatorState.CLIMBING, ArmState.CLIMBING);
        swerve.climbRequest(SnapUtil.getCageAngle(robotPose));
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.CLIMB_LINEUP);
        climber.setState(ClimberState.LINEUP_FORWARD);
      }
      case CLIMBING_2_HANGING -> {
        claw.setState(ClawState.IDLE_NO_GP);
        groundManager.climbRequest();
        moveSuperstructure(ElevatorState.CLIMBING, ArmState.CLIMBING);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.CLIMB_HANG);
        climber.setState(ClimberState.HANGING);
      }
      case CLIMBER_STOP -> {
        claw.setState(ClawState.IDLE_NO_GP);
        groundManager.climbRequest();
        moveSuperstructure(ElevatorState.CLIMBING, ArmState.CLIMBING);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.CLIMB_STOP);
        climber.setState(ClimberState.STOPPED);
      }
      case UNJAM -> {,
        claw.setState(ClawState.OUTTAKING);
        groundManager.unjamRequest();
        moveSuperstructure(ElevatorState.UNJAM, ArmState.UNJAM);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.OTHER);
        climber.setState(ClimberState.STOPPED);
      }
      case REHOME_ELEVATOR -> {
        claw.setState(ClawState.IDLE_NO_GP);
        moveSuperstructure(ElevatorState.MID_MATCH_HOMING, ArmState.HOLDING_UPRIGHT);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.OTHER);
        climber.setState(ClimberState.STOPPED);
      }
      case ALGAE_OUTTAKE -> {
        claw.setState(ClawState.OUTTAKING);
        moveSuperstructure(ElevatorState.ALGAE_OUTTAKE, ArmState.ALGAE_OUTTAKE);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.SCORING_ALGAE);
        climber.setState(ClimberState.STOPPED);
      }
      case PREPARE_SPIN_TO_WIN -> {
        claw.setState(ClawState.IDLE_NO_GP);
        moveSuperstructure(ElevatorState.PRE_CORAL_HANDOFF, ArmState.HOLDING_UPRIGHT);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.IDLE_EMPTY);
        climber.setState(ClimberState.STOPPED);
      }
      case SPIN_TO_WIN -> {
        claw.setState(ClawState.IDLE_NO_GP);
        moveSuperstructure(ElevatorState.PRE_CORAL_HANDOFF, ArmState.SPIN_TO_WIN, true);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.IDLE_EMPTY);
        climber.setState(ClimberState.STOPPED);
      }
      case ALGAE_FLING_WAIT -> {
        claw.setState(ClawState.IDLE_W_ALGAE);
        moveSuperstructure(ElevatorState.ALGAE_FLING, ArmState.ALGAE_FLING_WAIT);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.HOLDING_ALGAE);
        climber.setState(ClimberState.STOPPED);
      }
      case ALGAE_FLING_PREPARE -> {
        claw.setState(ClawState.IDLE_W_ALGAE);
        moveSuperstructure(ElevatorState.ALGAE_FLING, ArmState.ALGAE_FLING_SWING);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.HOLDING_ALGAE);
        climber.setState(ClimberState.STOPPED);
      }
      case ALGAE_FLING_RELEASE -> {
        claw.setState(ClawState.SCORE_ALGAE_NET);
        moveSuperstructure(ElevatorState.ALGAE_FLING, ArmState.ALGAE_FLING_SWING);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.SCORING_ALGAE);
        climber.setState(ClimberState.STOPPED);
      }
      case CLAW_ALGAE_STOW_INWARD -> {
        claw.setState(ClawState.IDLE_W_ALGAE);
        moveSuperstructure(ElevatorState.STOWED_INWARD, ArmState.STOWED_INWARD);
        swerve.normalDriveRequest();
        vision.setState(VisionState.TAGS);
        lights.setState(LightsState.HOLDING_ALGAE);
        climber.setState(ClimberState.STOPPED);
      }
    }
  }

  @Override
  public void robotPeriodic() {
    super.robotPeriodic();

    DogLog.log("RobotManager/NearestReefSidePose", nearestReefSide.getPose(robotPose));
    DogLog.log("CollisionAvoidance/latestUnsafe", latestUnsafe);
    DogLog.log("AutoAlign/ScoringLoopAroundObstruction", shouldLoopAroundToScoreObstruction);
    // Continuous state actions
    moveSuperstructure(latestElevatorGoal, latestArmGoal, latestUnsafe);

    arm.setLollipopMode(
        getState() == RobotState.CORAL_INTAKE_LOLLIPOP_GRAB
            || getState() == RobotState.CORAL_INTAKE_LOLLIPOP_APPROACH);

    switch (getState()) {
      case LOW_STOW, CLAW_ALGAE, STARTING_POSITION -> {
        if (groundManager.hasCoral() && !groundManager.deploy.atGoal()) {
          vision.setState(VisionState.HANDOFF);
        } else {
          vision.setState(VisionState.TAGS);
        }
        arm.setCoralHandoffOffset(vision.getHandoffOffsetTx());
      }
      case CLAW_EMPTY,
          CORAL_L4_PREPARE_HANDOFF,
          CORAL_L3_PREPARE_HANDOFF,
          CORAL_L2_PREPARE_HANDOFF -> {
        if (groundManager.hasCoral()) {
          vision.setState(VisionState.HANDOFF);
        } else {
          vision.setState(VisionState.TAGS);
        }
        arm.setCoralHandoffOffset(vision.getHandoffOffsetTx());
      }

      case CORAL_L1_RELEASE_HANDOFF,
          CORAL_L2_RELEASE_HANDOFF,
          CORAL_L3_RELEASE_HANDOFF,
          CORAL_L4_RELEASE_HANDOFF,
          CORAL_L1_AFTER_RELEASE_HANDOFF,
          CORAL_L2_AFTER_RELEASE_HANDOFF,
          CORAL_L3_AFTER_RELEASE_HANDOFF,
          CORAL_L4_AFTER_RELEASE_HANDOFF -> {
        // Do nothing, don't change the handoff angle when we are releasing
      }
      default -> arm.setCoralHandoffOffset(vision.getHandoffOffsetTx());
    }

    switch (getState()) {
      case ALGAE_INTAKE_L2_APPROACH,
          ALGAE_INTAKE_L3_APPROACH,
          ALGAE_INTAKE_L2,
          ALGAE_INTAKE_L3,
          ALGAE_INTAKE_L2_HOLDING,
          ALGAE_INTAKE_L3_HOLDING, -> {
        if (scoringAlignActive) {
          swerve.scoringAlignmentRequest(reefSnapAngle);
        } else {
          swerve.normalDriveRequest();
        }
        lights.setState(getLightStateFoAlgaeIntaking());
      }
      case CORAL_L1_PREPARE_HANDOFF,
          CORAL_L1_RELEASE_HANDOFF,
          CORAL_L1_AFTER_RELEASE_HANDOFF,
          CORAL_L2_PREPARE_HANDOFF,
          CORAL_L2_RELEASE_HANDOFF,
          CORAL_L2_AFTER_RELEASE_HANDOFF,
          CORAL_L3_PREPARE_HANDOFF,
          CORAL_L3_RELEASE_HANDOFF,
          CORAL_L3_AFTER_RELEASE_HANDOFF,
          CORAL_L4_PREPARE_HANDOFF,
          CORAL_L4_RELEASE_HANDOFF,
          CORAL_L4_AFTER_RELEASE_HANDOFF,
          CORAL_L1_APPROACH,
          CORAL_L2_APPROACH,
          CORAL_L3_PPROACH,
          CORAL_L4_APPROACH,
          CORAL_L2_LINEUP,
          CORAL_L3_LINEUP,
          CORAL_L4_LINEUP,
          CORAL_L2_PLACE,
          CORAL_L3_PLACE,
          CORAL_L4_PLACE,
          CORAL_L2_RELEASE,
          CORAL_L3_RELEASE,
          CORAL_L4_RELEASE -> {
        if (scoringAlignActive) {
          swerve.scoringAlignmentRequest(reefSnapAngle);
        } else {
          swerve.normalDriveRequest();
        }

        lights.setState(getLightStateForScoring());
      }
      case CORAL_INTAKE_LOLLIPOP_APPROACH,
          CORAL_INTAKE_LOLLIPOP_GRAB,
          CORAL_INTAKE_LOLLIPOP_PUSH -> {
        var lollipopVisionResult = vision.getLollipopVisionResult();
        coralMap.updateLollipopResult(lollipopVisionResult);
        lights.setState(
            lollipopVisionResult.isPresent()
                ? LightsState.LOLLIPOP_SEES_ALGAE
                : LightsState.LOLLIPOP_NO_ALGAE);
      }
      case CLAW_EMPTY -> {
        if (groundManager.getState() == GroundState.L1_WAIT
            || groundManager.getState().equals(GroundState.L1_HARD_WAIT)) {
          if (scoringAlignActive) {
            if (autoAlign.isTagAlignedDebounced()
                && FeatureFlags.AUTO_ALIGN_AUTO_SCORE.getAsBoolean()) {
              if (FeatureFlags.MANUAL_L1_HARD_SOFT.getAsBoolean()) {
                if (groundManager.getState() == GroundState.L1_WAIT) {
                  autoAlign.markPipeScored();
                  groundManager.l1Request();
                } else {
                  autoAlign.markPipeScored();
                  groundManager.hardL1Request();
                }
              } else {
                switch (autoAlign.getL1ScoredCount()) {
                  case 0 -> {
                    autoAlign.markPipeScored();
                    groundManager.l1Request();
                  }
                  case 1, 2 -> {
                    autoAlign.markPipeScored();
                    groundManager.hardL1Request();
                  }
                  default -> {
                    // Do nothing, we have scored enough
                  }
                }
              }
            }
            swerve.scoringAlignmentRequest(reefSnapAngle);
            lights.setState(getLightStateForScoring());
          } else {
            swerve.snapsDriveRequest(SnapUtil.getNearestReefAngle(robotPose));
          }
        } else {
          lights.setState(LightsState.IDLE_EMPTY);

          if (groundManager.hasCoral() && vision.isAnyTagLimelightOnline()) {
            swerve.snapsDriveRequest(
                MathHelpers.getDriveDirection(
                            robotPose,
                            new Pose2d(
                                AutoAlign.getAllianceCenterOfReef(robotPose), Rotation2d.kZero))
                        .getDegrees()
                    + (robotScoringSide.equals(RobotScoringSide.LEFT) ? 90 : -90));

          } else {
            swerve.normalDriveRequest();
          }
        }
      }
      case LOW_STOW, CLAW_ALGAE, STARTING_POSITION -> {
        if (groundManager.getState() == GroundState.L1_WAIT
            || groundManager.getState().equals(GroundState.L1_HARD_WAIT)) {
          if (scoringAlignActive) {
            if (autoAlign.isTagAlignedDebounced()) {
              if (FeatureFlags.MANUAL_L1_HARD_SOFT.getAsBoolean()) {
                if (groundManager.getState() == GroundState.L1_WAIT) {
                  autoAlign.markPipeScored();
                  groundManager.l1Request();
                } else {
                  autoAlign.markPipeScored();
                  groundManager.hardL1Request();
                }
              } else {
                switch (autoAlign.getL1ScoredCount()) {
                  case 0 -> {
                    autoAlign.markPipeScored();
                    groundManager.l1Request();
                  }
                  case 1, 2 -> {
                    autoAlign.markPipeScored();
                    groundManager.hardL1Request();
                  }
                  default -> {
                    // Do nothing, we have scored enough
                  }
                }
              }
            }
            swerve.scoringAlignmentRequest(reefSnapAngle);
            lights.setState(getLightStateForScoring());
          } else {
            swerve.snapsDriveRequest(SnapUtil.getNearestReefAngle(robotPose));
          }
        } else {
          lights.setState(LightsState.IDLE_EMPTY);

          swerve.normalDriveRequest();
        }
      }
      case CLAW_CORAL -> {
        if (vision.isAnyTagLimelightOnline()) {
          swerve.snapsDriveRequest(
              MathHelpers.getDriveDirection(
                          robotPose,
                          new Pose2d(
                              AutoAlign.getAllianceCenterOfReef(robotPose), Rotation2d.kZero))
                      .getDegrees()
                  + (robotScoringSide.equals(RobotScoringSide.LEFT) ? 90 : -90));
        } else {
          swerve.normalDriveRequest();
        }
      }
      default -> {}
    }

    if (FeatureFlags.FIELD_CALIBRATION.getAsBoolean()) {
      // Do nothing, field calibration will set the lights manually
    } else if (vision.isAnyCameraOffline()) {
      lights.setDisabledState(LightsState.ERROR);
    } else if (arm.getState() == ArmState.PRE_MATCH_HOMING && !arm.rangeOfMotionGood()) {
      lights.setDisabledState(LightsState.UNHOMED);
    } else if (vision.seeingTagDebounced()) {
      lights.setDisabledState(LightsState.HOMED_SEES_TAGS);
    } else {
      lights.setDisabledState(LightsState.HOMED_NO_TAGS);
    }

    switch (getState()) {
      case ALGAE_INTAKE_FLOOR -> {
        if (claw.getHasGP()) {
          rumbleController.rumbleRequest();
        }
      }
      default -> {}
    }

    autoAlign.setCoralL1Offset(vision.getHandoffOffsetTx());

    elevator.customPeriodic();
    arm.customPeriodic();
  }

  @Override
  protected void collectInputs() {
    if (DriverStation.isAutonomous()) {
      scoringAlignActive = true;
    }

    vision.setEstimatedPoseAngle(localization.getPose().getRotation().getDegrees());
    nearestReefSide = autoAlign.getClosestReefSide();
    robotPose = localization.getPose();

    robotScoringSide =
        AutoAlign.getScoringSideFromRobotPose(
            robotPose,
            vision.isAnyLeftScoringTagLimelightOnline(),
            vision.isAnyRightScoringTagLimelightOnline());
    shouldLoopAroundToScoreObstruction = autoAlign.getObstruction();
    reefSnapAngle = autoAlign.getUsedScoringPose().getRotation().getDegrees();
    scoringLevel =
        switch (getState()) {
          case CORAL_L1_PREPARE_HANDOFF,
              CORAL_L1_RELEASE_HANDOFF,
              CORAL_L1_AFTER_RELEASE_HANDOFF,
              CORAL_L2_PREPARE_HANDOFF,
              CORAL_L2_RELEASE_HANDOFF,
              CORAL_L2_AFTER_RELEASE_HANDOFF,
              CORAL_L3_PREPARE_HANDOFF,
              CORAL_L3_RELEASE_HANDOFF,
              CORAL_L3_AFTER_RELEASE_HANDOFF,
              CORAL_L4_PREPARE_HANDOFF,
              CORAL_L4_RELEASE_HANDOFF,
              CORAL_L4_AFTER_RELEASE_HANDOFF,
              CORAL_L1_RIGHT_APPROACH,
              CORAL_L2_LEFT_APPROACH,
              CORAL_L2_RIGHT_APPROACH,
              CORAL_L3_LEFT_APPROACH,
              CORAL_L3_RIGHT_APPROACH,
              CORAL_L4_LEFT_APPROACH,
              CORAL_L4_RIGHT_APPROACH ->
              ReefPipeLevel.RAISING;

          case CORAL_L2_LEFT_RELEASE,
              CORAL_L2_RIGHT_RELEASE,
              CORAL_L3_LEFT_RELEASE,
              CORAL_L3_RIGHT_RELEASE,
              CORAL_L4_LEFT_RELEASE,
              CORAL_L4_RIGHT_RELEASE ->
              ReefPipeLevel.BACK_AWAY;

          case LOW_STOW -> ReefPipeLevel.L1;
          case CORAL_L2_LEFT_LINEUP,
              CORAL_L2_LEFT_PLACE,
              CORAL_L2_RIGHT_LINEUP,
              CORAL_L2_RIGHT_PLACE ->
              ReefPipeLevel.L2;
          case CORAL_L3_LEFT_LINEUP,
              CORAL_L3_LEFT_PLACE,
              CORAL_L3_RIGHT_LINEUP,
              CORAL_L3_RIGHT_PLACE ->
              ReefPipeLevel.L3;
          case CORAL_L4_LEFT_LINEUP,
              CORAL_L4_LEFT_PLACE,
              CORAL_L4_RIGHT_LINEUP,
              CORAL_L4_RIGHT_PLACE ->
              ReefPipeLevel.L4;
          default -> ReefPipeLevel.RAISING;
        };

    var preferredScoringLevel =
        switch (getState()) {
          case CORAL_L1_PREPARE_HANDOFF,
              CORAL_L1_RELEASE_HANDOFF,
              CORAL_L1_RIGHT_APPROACH,
              CORAL_L1_RIGHT_LINEUP,
              CORAL_L1_RIGHT_RELEASE ->
              ReefPipeLevel.RAISING;
          case CORAL_L2_PREPARE_HANDOFF,
              CORAL_L2_RELEASE_HANDOFF,
              CORAL_L2_AFTER_RELEASE_HANDOFF,
              CORAL_L2_LEFT_APPROACH,
              CORAL_L2_LEFT_LINEUP,
              CORAL_L2_LEFT_PLACE,
              CORAL_L2_LEFT_RELEASE,
              CORAL_L2_RIGHT_APPROACH,
              CORAL_L2_RIGHT_LINEUP,
              CORAL_L2_RIGHT_PLACE,
              CORAL_L2_RIGHT_RELEASE ->
              ReefPipeLevel.L2;
          case CORAL_L3_PREPARE_HANDOFF,
              CORAL_L3_RELEASE_HANDOFF,
              CORAL_L3_AFTER_RELEASE_HANDOFF,
              CORAL_L3_LEFT_APPROACH,
              CORAL_L3_LEFT_LINEUP,
              CORAL_L3_LEFT_PLACE,
              CORAL_L3_LEFT_RELEASE,
              CORAL_L3_RIGHT_APPROACH,
              CORAL_L3_RIGHT_LINEUP,
              CORAL_L3_RIGHT_PLACE,
              CORAL_L3_RIGHT_RELEASE ->
              ReefPipeLevel.L3;
          case CORAL_L4_PREPARE_HANDOFF,
              CORAL_L4_RELEASE_HANDOFF,
              CORAL_L4_AFTER_RELEASE_HANDOFF,
              CORAL_L4_LEFT_APPROACH,
              CORAL_L4_LEFT_LINEUP,
              CORAL_L4_LEFT_PLACE,
              CORAL_L4_LEFT_RELEASE,
              CORAL_L4_RIGHT_APPROACH,
              CORAL_L4_RIGHT_LINEUP,
              CORAL_L4_RIGHT_PLACE,
              CORAL_L4_RIGHT_RELEASE ->
              ReefPipeLevel.L4;
          case LOW_STOW -> ReefPipeLevel.L1; // Prefer L1 when stowing low
          default -> {
            yield switch (groundManager.getState()) {
              case L1_WAIT, L1_SCORE, L1_HARD_SCORE -> ReefPipeLevel.L1;
              default -> ReefPipeLevel.RAISING;
            };
          }
        };

    autoAlign.setScoringLevel(scoringLevel, preferredScoringLevel, robotScoringSide);

    var reefSideOffset =
        switch (getState()) {
          case ALGAE_INTAKE_L2_LEFT,
              ALGAE_INTAKE_L3_LEFT,
              ALGAE_INTAKE_L2_RIGHT,
              ALGAE_INTAKE_L3_RIGHT ->
              ReefSideOffset.ALGAE_INTAKING;
          default -> ReefSideOffset.ALGAE_RAISING;
        };
    autoAlign.setAlgaeIntakingOffset(reefSideOffset);

    vision.setClosestScoringReefAndPipe(nearestReefSide.getTagID());

    if (vision.isAnyTagLimelightOnline() && DriverStation.isTeleop()) {
      var idealAlignSpeeds =
          switch (getState()) {
            case ALGAE_INTAKE_L2_LEFT_APPROACH,
                ALGAE_INTAKE_L3_LEFT_APPROACH,
                ALGAE_INTAKE_L2_RIGHT_APPROACH,
                ALGAE_INTAKE_L3_RIGHT_APPROACH,
                ALGAE_INTAKE_L2_LEFT,
                ALGAE_INTAKE_L3_LEFT,
                ALGAE_INTAKE_L2_RIGHT,
                ALGAE_INTAKE_L3_RIGHT,
                ALGAE_INTAKE_L2_LEFT_HOLDING,
                ALGAE_INTAKE_L2_RIGHT_HOLDING,
                ALGAE_INTAKE_L3_LEFT_HOLDING,
                ALGAE_INTAKE_L3_RIGHT_HOLDING ->
                autoAlign.getAlgaeAlignSpeeds();
            default -> autoAlign.getTagAlignSpeeds();
          };
      swerve.setAutoAlignSpeeds(idealAlignSpeeds);
    } else {
      swerve.setAutoAlignSpeeds(swerve.getTeleopSpeeds());
    }

    var maybeCoralPose = coralMap.getBestCoralPose();
    if (maybeCoralPose.isPresent()) {

      var coralAlignSpeeds =
          IntakeAssistUtil.getAssistSpeedsFromPose(
              maybeCoralPose.get(), robotPose, swerve.getTeleopSpeeds());
      swerve.setFieldRelativeCoralAssistSpeeds(coralAlignSpeeds);
    } else {
      swerve.setFieldRelativeCoralAssistSpeeds(swerve.getTeleopSpeeds());
    }

    swerve.setElevatorHeight(elevator.getHeight());
  }

  private boolean cameraOnlineAndFarEnoughFromReef() {
    var tagCameraOnline = vision.isAnyTagLimelightOnline();

    if (!tagCameraOnline) {
      return timeout(0.5);
    }

    var isFarEnoughFromReefSide =
        !AutoAlign.isCloseToReefSide(robotPose, nearestReefSide.getPose(robotPose), 1.0);

    return isFarEnoughFromReefSide;
  }

  private boolean drivingAwayFromReef() {
    var tagCameraOnline = vision.isAnyTagLimelightOnline();

    if (!tagCameraOnline) {
      return timeout(1.0);
    }

    var isFarEnoughFromReefSide =
        !AutoAlign.isCloseToReefSide(robotPose, nearestReefSide.getPose(robotPose), 1.0);

    var speeds = swerve.getTeleopSpeeds();
    var isDrivingAway = Math.hypot(speeds.vxMetersPerSecond, speeds.vyMetersPerSecond) > 0.5;
    return isFarEnoughFromReefSide && isDrivingAway;
  }

  public void intakeRequest() {
    switch (getState()) {
      case LOW_STOW -> {
        if (groundManager.getState().equals(GroundState.L1_WAIT) && scoringAlignActive) {
          groundManager.hardL1WaitRequest();
        } else {
          intakeAssistActive = true;
          groundManager.intakeRequest();
        }
      }
      default -> groundManager.intakeRequest();
    }
  }

  public void hardL1OffRequest() {
    intakeAssistActive = false;
    switch (getState()) {
      case LOW_STOW -> {
        if (groundManager.getState().equals(GroundState.L1_HARD_WAIT)) {
          groundManager.l1WaitRequest();
        }
      }
      default -> {}
    }
  }

  public void forceIdleNoGp() {
    setStateFromRequest(RobotState.CLAW_EMPTY);
  }

  public void stowRequest() {
    afterIntakingCoralState = Optional.empty();
    groundManager.idleRequest();
    switch (getState()) {
      case ALGAE_INTAKE_L2_LEFT,
          ALGAE_INTAKE_L2_RIGHT,
          ALGAE_INTAKE_L3_LEFT,
          LOW_STOW,
          ALGAE_INTAKE_L3_RIGHT ->
          setStateFromRequest(RobotState.CLAW_EMPTY);
      default -> {
        if (claw.getHasGP()) {
          // Claw is maybe algae or coral
          if (getState().clawGp == ClawGamePiece.ALGAE) {
            setStateFromRequest(RobotState.CLAW_ALGAE);
          } else {
            setStateFromRequest(RobotState.CLAW_CORAL);
          }
        } else {
          setStateFromRequest(RobotState.CLAW_EMPTY);
        }
      }
    }
  }

  public void forcedLowStowRequest() {
    if (getState().climbingOrRehoming) {
      return;
    }

    setStateFromRequest(RobotState.FORCED_LOWSTOW);
  }

  public void forcedHandoffRequest() {
    if (getState().climbingOrRehoming) {
      return;
    }

    setStateFromRequest(RobotState.FORCED_HANDOFF);
  }

  public void algaeFlingRequest() {
    if (getState().climbingOrRehoming) {
      return;
    }

    setStateFromRequest(RobotState.ALGAE_FLING_WAIT);
  }

  public void algaeFlingConfirmRequest() {
    if (getState().climbingOrRehoming) {
      return;
    }

    setStateFromRequest(RobotState.ALGAE_FLING_PREPARE);
  }

  public void intakeFloorAlgaeRequest() {
    if (!getState().climbingOrRehoming) {
      if (getState() == RobotState.CLAW_EMPTY) {
        if (elevator.atGoal() && arm.atGoal()) {
          // Ignore algae intake until stow motion is finished
          setStateFromRequest(RobotState.ALGAE_INTAKE_FLOOR);
        }
      } else {
        setStateFromRequest(RobotState.ALGAE_INTAKE_FLOOR);
      }
    }
  }

  public void lollipopIntakeApproachRequest() {
    if (!getState().climbingOrRehoming) {
      setStateFromRequest(RobotState.CORAL_INTAKE_LOLLIPOP_APPROACH);
    }
  }

  public void lollipopIntakeGrabRequest() {
    if (!getState().climbingOrRehoming) {
      setStateFromRequest(RobotState.CORAL_INTAKE_LOLLIPOP_GRAB);
    }
  }

  public void testingNextLollipopRequest() {
    if (!getState().climbingOrRehoming) {
      switch (getState()) {
        case CORAL_INTAKE_LOLLIPOP_APPROACH -> lollipopIntakeApproachRequest();
        case CORAL_INTAKE_LOLLIPOP_GRAB ->
            setStateFromRequest(RobotState.CORAL_INTAKE_LOLLIPOP_PUSH);
        case CORAL_INTAKE_LOLLIPOP_PUSH -> setStateFromRequest(RobotState.STARTING_POSITION_CORAL);
        default -> lollipopIntakeGrabRequest();
      }
    }
  }

  public void processorWaitingRequest() {
    if (!getState().climbingOrRehoming) {
      setStateFromRequest(RobotState.ALGAE_PROCESSOR_WAITING);
    }
  }

  public void scoringAlignOffRequest() {
    scoringAlignActive = false;
  }

  public void l4CoralLeftAutoApproachRequest() {
    if (DriverStation.isAutonomous()) {
      setStateFromRequest(RobotState.CORAL_L4_LEFT_APPROACH);
    }
  }

  public void l4CoralRightAutoApproachRequest() {
    if (DriverStation.isAutonomous()) {
      setStateFromRequest(RobotState.CORAL_L4_RIGHT_APPROACH);
    }
  }

  public void l3CoralLeftAutoApproachRequest() {
    if (DriverStation.isAutonomous()) {
      setStateFromRequest(RobotState.CORAL_L3_LEFT_APPROACH);
    }
  }

  public void l3CoralRightAutoApproachRequest() {
    if (DriverStation.isAutonomous()) {
      setStateFromRequest(RobotState.CORAL_L3_RIGHT_APPROACH);
    }
  }

  public void l2CoralLeftAutoApproachRequest() {
    if (DriverStation.isAutonomous()) {
      setStateFromRequest(RobotState.CORAL_L2_LEFT_APPROACH);
    }
  }

  public void l2CoralRightAutoApproachRequest() {
    if (DriverStation.isAutonomous()) {
      setStateFromRequest(RobotState.CORAL_L2_RIGHT_APPROACH);
    }
  }

  public void l2CoralLeftAutoLineupRequest() {
    if (DriverStation.isAutonomous()) {
      setStateFromRequest(RobotState.CORAL_L2_LEFT_LINEUP);
    }
  }

  public void l2CoralRightAutoLineupRequest() {
    if (DriverStation.isAutonomous()) {
      setStateFromRequest(RobotState.CORAL_L2_RIGHT_LINEUP);
    }
  }

  public void l4CoralApproachRequest() {
    if (getState().climbingOrRehoming) {
      return;
    }
    scoringAlignActive = true;

    if (claw.getHasGP() || RobotState.isLineupOrApproachState(getState())) {

      switch (robotScoringSide) {
        case LEFT -> setStateFromRequest(RobotState.CORAL_L4_LEFT_APPROACH);
        case RIGHT -> setStateFromRequest(RobotState.CORAL_L4_RIGHT_APPROACH);
      }
    } else {
      setStateFromRequest(RobotState.CORAL_L4_PREPARE_HANDOFF);
    }
  }

  public void l3CoralApproachRequest() {
    if (getState().climbingOrRehoming) {
      return;
    }

    scoringAlignActive = true;

    if (claw.getHasGP() || RobotState.isLineupOrApproachState(getState())) {
      switch (robotScoringSide) {
        case LEFT -> setStateFromRequest(RobotState.CORAL_L3_LEFT_APPROACH);
        case RIGHT -> setStateFromRequest(RobotState.CORAL_L3_RIGHT_APPROACH);
      }
    } else {
      setStateFromRequest(RobotState.CORAL_L3_PREPARE_HANDOFF);
    }
  }

  public void l2CoralApproachRequest() {
    if (getState().climbingOrRehoming) {
      return;
    }

    scoringAlignActive = true;

    if (claw.getHasGP() || RobotState.isLineupOrApproachState(getState())) {
      switch (robotScoringSide) {
        case LEFT -> setStateFromRequest(RobotState.CORAL_L2_LEFT_APPROACH);
        case RIGHT -> setStateFromRequest(RobotState.CORAL_L2_RIGHT_APPROACH);
      }
    } else {
      setStateFromRequest(RobotState.CORAL_L2_PREPARE_HANDOFF);
    }
  }

  public void l1CoralApproachRequest() {
    if (getState().climbingOrRehoming) {
      return;
    }

    if (claw.getHasGP() || RobotState.isLineupOrApproachState(getState())) {
      setStateFromRequest(RobotState.CORAL_L1_RIGHT_APPROACH);
    } else {
      setStateFromRequest(RobotState.CORAL_L1_PREPARE_HANDOFF);
    }
  }

  public void highLineupRequest() {
    if (!getState().climbingOrRehoming) {
      if (getState().clawGp == ClawGamePiece.ALGAE) {
        algaeNetRequest();
      } else {
        l4CoralApproachRequest();
      }
    }
  }

  public void lowLineupRequest() {
    if (!getState().climbingOrRehoming) {
      if (getState().clawGp == ClawGamePiece.ALGAE) {
        processorWaitingRequest();
      } else {
        l1CoralApproachRequest();
      }
    }
  }

  public void l4CoralLeftReleaseRequest() {
    if (!getState().climbingOrRehoming) {
      if (claw.getHasGP()) {
        setStateFromRequest(RobotState.CORAL_L4_LEFT_RELEASE);
      } else {
        setStateFromRequest(RobotState.CORAL_L4_PREPARE_HANDOFF);
      }
    }
  }

  public void l3CoralLeftReleaseRequest() {
    if (!getState().climbingOrRehoming) {
      if (claw.getHasGP()) {
        setStateFromRequest(RobotState.CORAL_L3_LEFT_RELEASE);
      } else {
        setStateFromRequest(RobotState.CORAL_L3_PREPARE_HANDOFF);
      }
    }
  }

  public void algaeReefIntakeRequest() {
    if (!getState().climbingOrRehoming) {
      scoringAlignActive = true;
      if (robotScoringSide == RobotScoringSide.LEFT) {

        switch (getState()) {
          case ALGAE_INTAKE_L2_LEFT_APPROACH,
              ALGAE_INTAKE_L2_LEFT,
              ALGAE_INTAKE_L2_LEFT_HOLDING,
              ALGAE_INTAKE_L3_LEFT_APPROACH,
              ALGAE_INTAKE_L3_LEFT,
              ALGAE_INTAKE_L3_LEFT_HOLDING -> {
            // Do nothing, prevent multiple presses from forcing collision avoidance to trigger
          }
          case CORAL_L2_LEFT_PLACE,
              CORAL_L2_LEFT_RELEASE,
              CORAL_L3_LEFT_PLACE,
              CORAL_L3_LEFT_RELEASE,
              CORAL_L4_LEFT_PLACE,
              CORAL_L4_LEFT_RELEASE ->
              canSkipCollisionAvoidanceForReefAlgae = true;
          default -> canSkipCollisionAvoidanceForReefAlgae = false;
        }

        if (nearestReefSide.algaeHeight == ReefPipeLevel.L3) {
          setStateFromRequest(RobotState.ALGAE_INTAKE_L3_LEFT_APPROACH);
        } else {
          setStateFromRequest(RobotState.ALGAE_INTAKE_L2_LEFT_APPROACH);
        }
      } else {
        switch (getState()) {
          case ALGAE_INTAKE_L2_RIGHT_APPROACH,
              ALGAE_INTAKE_L2_RIGHT,
              ALGAE_INTAKE_L2_RIGHT_HOLDING,
              ALGAE_INTAKE_L3_RIGHT_APPROACH,
              ALGAE_INTAKE_L3_RIGHT,
              ALGAE_INTAKE_L3_RIGHT_HOLDING -> {
            // Do nothing, prevent multiple presses from forcing collision avoidance to trigger
          }
          case CORAL_L2_RIGHT_PLACE,
              CORAL_L2_RIGHT_RELEASE,
              CORAL_L3_RIGHT_PLACE,
              CORAL_L3_RIGHT_RELEASE,
              CORAL_L4_RIGHT_PLACE,
              CORAL_L4_RIGHT_RELEASE ->
              canSkipCollisionAvoidanceForReefAlgae = true;
          default -> canSkipCollisionAvoidanceForReefAlgae = false;
        }

        if (nearestReefSide.algaeHeight == ReefPipeLevel.L3) {
          setStateFromRequest(RobotState.ALGAE_INTAKE_L3_RIGHT_APPROACH);
        } else {
          setStateFromRequest(RobotState.ALGAE_INTAKE_L2_RIGHT_APPROACH);
        }
      }
    }
  }

  public void algaeNetRequest() {
    if (!vision.isAnyTagLimelightOnline()
        || AutoAlign.getNetScoringSideFromRobotPose(robotPose) == RobotScoringSide.LEFT) {
      algaeNetLeftRequest();
    } else {
      algaeNetRightRequest();
    }
  }

  private void algaeNetLeftRequest() {
    if (!getState().climbingOrRehoming) {
      setStateFromRequest(RobotState.ALGAE_NET_LEFT_WAITING);
    }
  }

  private void algaeNetRightRequest() {
    if (!getState().climbingOrRehoming) {
      setStateFromRequest(RobotState.ALGAE_NET_RIGHT_WAITING);
    }
  }

  public void preloadCoralRequest() {
    if (!getState().climbingOrRehoming) {
      setStateFromRequest(RobotState.STARTING_POSITION_CORAL);
    }
  }

  public void startingPositionRequest() {
    if (!getState().climbingOrRehoming) {
      setStateFromRequest(RobotState.STARTING_POSITION);
    }
  }

  public void confirmScoreRequest() {
    switch (getState()) {
      case CLIMBING_1_LINEUP,
          CLIMBING_2_HANGING,
          CLIMBER_STOP,
          CORAL_L1_PREPARE_HANDOFF,
          CORAL_L2_PREPARE_HANDOFF,
          CORAL_L3_PREPARE_HANDOFF,
          CORAL_L4_PREPARE_HANDOFF,
          CORAL_L1_RELEASE_HANDOFF,
          CORAL_L2_RELEASE_HANDOFF,
          CORAL_L3_RELEASE_HANDOFF,
          CORAL_L4_RELEASE_HANDOFF,
          CORAL_L1_AFTER_RELEASE_HANDOFF,
          CORAL_L2_AFTER_RELEASE_HANDOFF,
          CORAL_L3_AFTER_RELEASE_HANDOFF,
          CORAL_L4_AFTER_RELEASE_HANDOFF -> {}

      case CORAL_L1_RIGHT_APPROACH,
          CORAL_L2_LEFT_APPROACH,
          CORAL_L2_RIGHT_APPROACH,
          CORAL_L3_LEFT_APPROACH,
          CORAL_L3_RIGHT_APPROACH,
          CORAL_L4_LEFT_APPROACH,
          CORAL_L4_RIGHT_APPROACH ->
          setStateFromRequest(getState().getLeftApproachToLineupState());

      case ALGAE_FLING_WAIT -> setStateFromRequest(RobotState.ALGAE_FLING_PREPARE);

      case CORAL_L2_LEFT_LINEUP,
          CORAL_L3_LEFT_LINEUP,
          CORAL_L4_LEFT_LINEUP,
          CORAL_L1_RIGHT_LINEUP,
          CORAL_L2_RIGHT_LINEUP,
          CORAL_L3_RIGHT_LINEUP,
          CORAL_L4_RIGHT_LINEUP -> {
        setStateFromRequest(getState().getLineupToPlaceState());
      }

      case CORAL_L2_LEFT_PLACE,
          CORAL_L3_LEFT_PLACE,
          CORAL_L4_LEFT_PLACE,
          CORAL_L2_RIGHT_PLACE,
          CORAL_L3_RIGHT_PLACE,
          CORAL_L4_RIGHT_PLACE -> {
        autoAlign.markPipeScored();
        setStateFromRequest(getState().getPlaceToReleaseState());
      }

      case CLAW_ALGAE -> {
        if (groundManager.hasCoral()) {
          groundManager.l1Request();
          scoringAlignActive = true;
        } else {
          setStateFromRequest(RobotState.ALGAE_OUTTAKE);
        }
      }
      case CLAW_EMPTY, STARTING_POSITION -> {
        if (groundManager.hasCoral()) {
          endgameStowRequest();
          groundManager.l1Request();
          scoringAlignActive = true;
        } else {
          setStateFromRequest(RobotState.ALGAE_OUTTAKE);
        }
      }

      case CLAW_CORAL, STARTING_POSITION_CORAL -> l4CoralApproachRequest();
      case LOW_STOW -> {
        groundManager.l1Request();
        scoringAlignActive = true;
      }

      case ALGAE_PROCESSOR_WAITING -> setStateFromRequest(RobotState.ALGAE_PROCESSOR_RELEASE);

      case ALGAE_NET_LEFT_WAITING -> setStateFromRequest(RobotState.ALGAE_NET_LEFT_RELEASE);

      case ALGAE_NET_RIGHT_WAITING -> setStateFromRequest(RobotState.ALGAE_NET_RIGHT_RELEASE);

      default -> {}
    }
  }

  public void nextClimbStateRequest() {
    switch (getState()) {
      case CLIMBER_STOP -> setStateFromRequest(RobotState.CLIMBING_1_LINEUP);
      case STARTING_POSITION, LOW_STOW, CLAW_EMPTY, CLAW_CORAL, CLAW_ALGAE -> {
        if (arm.atGoal() && elevator.atGoal()) {
          setStateFromRequest(RobotState.CLIMBING_1_LINEUP);
        }
      }
      case CLIMBING_1_LINEUP -> setStateFromRequest(RobotState.CLIMBING_2_HANGING);
      case CLIMBING_2_HANGING -> {}
      default -> {
        // Do nothing
      }
    }
  }

  public void stopClimbStateRequest() {
    switch (getState()) {
      case CLIMBING_1_LINEUP, CLIMBING_2_HANGING -> setStateFromRequest(RobotState.CLIMBER_STOP);
      default -> {}
    }
  }

  public void unjamRequest() {
    if (!getState().climbingOrRehoming) {
      setStateFromRequest(RobotState.UNJAM);
    }
  }

  public void lowStowRequest() {
    if (!getState().climbingOrRehoming) {
      setStateFromRequest(RobotState.REHOME_ELEVATOR);
    }
  }

  public void spinToWinRequest() {
    if (!getState().climbingOrRehoming && FeatureFlags.SPIN_TO_WIN.getAsBoolean()) {
      setStateFromRequest(RobotState.PREPARE_SPIN_TO_WIN);
    }
  }

  public void endgameStowRequest() {
    if (!getState().climbingOrRehoming) {
      setStateFromRequest(RobotState.LOW_STOW);
    }
  }

  private ElevatorState latestElevatorGoal = ElevatorState.STOWED;
  private ArmState latestArmGoal = ArmState.HOLDING_UPRIGHT;
  private boolean latestUnsafe = false;

  private void moveSuperstructure(ElevatorState elevatorGoal, ArmState armGoal) {
    moveSuperstructure(elevatorGoal, armGoal, false);
  }

  private void moveSuperstructure(ElevatorState elevatorGoal, ArmState armGoal, boolean unsafe) {
    latestElevatorGoal = elevatorGoal;
    latestArmGoal = armGoal;
    latestUnsafe = unsafe;

    var currentPosition = new SuperstructurePosition(elevator.getHeight(), arm.getAngle());
    var goal = new SuperstructurePosition(elevatorGoal.getHeight(), armGoal.getAngle());

    MechanismVisualizer.log(currentPosition, groundManager.deploy.getAngle());
    var obstructionKind =
        FeatureFlags.COLLISION_AVOIDANCE_OBSTRUCTION.getAsBoolean()
            ? shouldLoopAroundToScoreObstruction
            : ObstructionKind.NONE;
    var maybeCollisionAvoidanceResult =
        CollisionAvoidance.routePosition(currentPosition, goal, obstructionKind, arm.getRawAngle());

    DogLog.log("CollisionAvoidance/LatestResultPresent", maybeCollisionAvoidanceResult.isPresent());

    if (unsafe || maybeCollisionAvoidanceResult.isEmpty()) {
      elevator.setState(elevatorGoal);
      arm.setState(armGoal);
    } else {
      var collisionAvoidanceResult = maybeCollisionAvoidanceResult.orElseThrow();

      elevator.setCollisionAvoidanceGoal(collisionAvoidanceResult.elevatorHeight());
      elevator.setState(ElevatorState.COLLISION_AVOIDANCE);

      arm.setCollisionAvoidanceGoal(collisionAvoidanceResult.armAngle());
      arm.setState(ArmState.COLLISION_AVOIDANCE);
    }
  }

  private LightsState getLightStateFoAlgaeIntaking() {
    if (!vision.isAnyTagLimelightOnline()) {
      return LightsState.ERROR;
    }

    if (claw.getHasGP()) {
      return LightsState.SCORE_ALIGN_TAGS;
    }
    return LightsState.SCORE_NO_ALIGN_TAGS;
  }

  private LightsState getLightStateForScoring() {
    return switch (autoAlign.getReefAlignState()) {
      case ALL_CAMERAS_DEAD -> LightsState.ERROR;
      case HAS_TAGS_IN_POSITION, HAS_TAGS_WRONG_POSITION ->
          scoringAlignActive ? LightsState.SCORE_ALIGN_TAGS : LightsState.SCORE_NO_ALIGN_TAGS;
      case NO_TAGS_IN_POSITION, NO_TAGS_WRONG_POSITION ->
          scoringAlignActive ? LightsState.SCORE_ALIGN_NO_TAGS : LightsState.SCORE_NO_ALIGN_NO_TAGS;
    };
  }
}
