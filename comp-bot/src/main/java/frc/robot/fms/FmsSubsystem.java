package frc.robot.fms;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import frc.robot.util.scheduling.LifecycleSubsystem;
import frc.robot.util.scheduling.SubsystemPriority;

public class FmsSubsystem extends LifecycleSubsystem {
  public FmsSubsystem() {
    super(SubsystemPriority.FMS);
  }

  public static boolean isRedAlliance() {
    Alliance alliance = DriverStation.getAlliance().orElse(Alliance.Red);

    return alliance == Alliance.Red;
  }

  @Override
  public void robotPeriodic() {
    DogLog.log("Fms/Alliance", isRedAlliance() ? "Red" : "Blue");
  }
}
