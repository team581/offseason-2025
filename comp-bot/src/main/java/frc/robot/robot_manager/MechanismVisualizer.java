package frc.robot.robot_manager;

import com.team581.GlobalConfig;
import dev.doglog.DogLog;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc.robot.arm.ArmSubsystem;
import frc.robot.config.RobotConfig;

public final class MechanismVisualizer {
  private static final Translation2d MECHANISM_AREA =
      new Translation2d(
          ((ArmSubsystem.ARM_LENGTH_METERS + Units.inchesToMeters(2)) * 2.0),
          Units.inchesToMeters(RobotConfig.get().elevator().maxHeight() + 2)
              + ArmSubsystem.ARM_LENGTH_METERS);
  private static final Mechanism2d mechanism =
      new Mechanism2d(
          MECHANISM_AREA.getX(), MECHANISM_AREA.getY(), new Color8Bit(new Color("#121212")));
  private static final MechanismRoot2d root =
      mechanism.getRoot("superstructure", MECHANISM_AREA.getX() / 2.0, Units.inchesToMeters(2));
  private static final MechanismLigament2d elevator =
      root.append(
          new MechanismLigament2d(
              "elevator",
              Units.inchesToMeters(RobotConfig.get().elevator().minHeight()),
              90,
              20,
              new Color8Bit(Color.kFirstBlue)));
  private static final MechanismLigament2d arm =
      elevator.append(
          new MechanismLigament2d(
              "arm", ArmSubsystem.ARM_LENGTH_METERS, 90, 10, new Color8Bit(Color.kFirstRed)));

  public static void log(SuperstructurePosition position, double deployAngle) {
    if (!GlobalConfig.IS_DEVELOPMENT) {
      return;
    }

    SmartDashboard.putData("SuperstructureVisualization", mechanism);

    var elevatorHeight = Units.inchesToMeters(position.elevatorHeight());
    var armAngle = -1.0 * (position.armAngle() + 360 - elevator.getAngle());

    var elevatorPose = new Pose3d(Translation3d.kZero, Rotation3d.kZero);
    var carriagePose = new Pose3d(new Translation3d(0, 0, elevatorHeight), Rotation3d.kZero);
    var armPose =
        new Pose3d(
            new Translation3d(0, 0, elevatorHeight),
            new Rotation3d(Units.degreesToRadians(armAngle), 0, 0));
    var deployPose =
        new Pose3d(
            Translation3d.kZero, new Rotation3d(0, -1.0 * Units.degreesToRadians(deployAngle), 0));
    DogLog.log(
        "SuperstructureVisualization/Superstructure3d",
        new Pose3d[] {elevatorPose, carriagePose, armPose, deployPose});

    elevator.setLength(Units.inchesToMeters(position.elevatorHeight()));
    arm.setAngle(armAngle);
  }

  private MechanismVisualizer() {}
}
