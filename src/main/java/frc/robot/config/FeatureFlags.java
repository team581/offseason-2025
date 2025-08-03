package frc.robot.config;

import frc.robot.util.FeatureFlag;
import java.util.function.BooleanSupplier;

public class FeatureFlags {
  public static final BooleanSupplier PURE_PURSUIT_USE_DYNAMIC_LOOKAHEAD =
      FeatureFlag.of("PurePursuit/UseDynamicLookahead", false);

  public static final BooleanSupplier PURE_PURSUIT_ROTATE_IMMEDIATELY =
      FeatureFlag.of("PurePursuit/RotateImmediately", false);

  public static final BooleanSupplier AUTO_ALIGN_DEADBAND =
      FeatureFlag.of("AutoAlign/Deadband", false);

  public static final BooleanSupplier CAMERA_POSITION_CALIBRATION =
      FeatureFlag.of("Vision/PositionCalibrationMode", false);

  public static final BooleanSupplier CORAL_DETECTION = FeatureFlag.of("CoralMap", true);

  public static final BooleanSupplier VISION_STALE_DATA_CHECK =
      FeatureFlag.of("Vision/StaleDataRejection", false);

  public static final BooleanSupplier MT_VISION_METHOD =
      FeatureFlag.of("Vision/MTVisionMethod", true);

  public static final BooleanSupplier FIELD_CALIBRATION = FeatureFlag.of("FieldCalibration", false);

  public static final BooleanSupplier COLLISION_AVOIDANCE_OBSTRUCTION =
      FeatureFlag.of("CollisionAvoidance/Obstruction", false);

  public static final BooleanSupplier USE_ALTERNATE_WAYPOINT_CHOOSER =
      FeatureFlag.of("CollisionAvoidance/AlternateClosestWaypointChooser", true);

  public static final BooleanSupplier MANUAL_L1_HARD_SOFT =
      FeatureFlag.of("Controls/ManualL1HardSoft", false);

  public static final BooleanSupplier APPROACH_TAG_CHECK =
      FeatureFlag.of("Vision/ApproachTagCheck", true);

  public static final BooleanSupplier AUTO_STOW_ALGAE = FeatureFlag.of("AutoStowAlgaeScore", false);

  public static final BooleanSupplier AUTO_ALIGN_AUTO_SCORE =
      FeatureFlag.of("AutoAlign/AutoScore", true);
  public static final BooleanSupplier AUTO_ALIGN_MAX_ROTATION_LIMIT =
      FeatureFlag.of("AutoAlign/MaxRotationLimit", false);

  public static final BooleanSupplier AUTO_ALIGN_DISTANCE_COST =
      FeatureFlag.of("AutoAlign/Costs/Distance", true);
  public static final BooleanSupplier AUTO_ALIGN_DRIVE_DIRECTION_COST =
      FeatureFlag.of("AutoAlign/Costs/DriveDirection", true);
  public static final BooleanSupplier AUTO_ALIGN_HEADING_COST =
      FeatureFlag.of("AutoAlign/Costs/Heading", false);
  public static final BooleanSupplier AUTO_ALIGN_REEF_STATE_COST =
      FeatureFlag.of("AutoAlign/Costs/ReefState", true);

  public static final BooleanSupplier SPIN_TO_WIN = FeatureFlag.of("Yapping/SpinToWin", false);

  public static final BooleanSupplier USE_ANY_REEF_TAG =
      FeatureFlag.of("Vision/UseAnyReefTag", true);

  private FeatureFlags() {}
}
