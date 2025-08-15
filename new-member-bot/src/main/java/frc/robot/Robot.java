package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Robot extends TimedRobot {
  private static final String kDefaultAuto = "Default";
  private static final String kCustomAuto = "My Auto";
  private String mAutoSelected;
  private final SendableChooser<String> mChooser = new SendableChooser<>();

  @Override
  public void robotInit() {
    mChooser.setDefaultOption("Default Auto", kDefaultAuto);
    mChooser.addOption("My Auto", kCustomAuto);
    SmartDashboard.putData("Auto choices", mChooser);
  }

  @Override
  public void robotPeriodic() {}

  @Override
  public void autonomousInit() {
    mAutoSelected = mChooser.getSelected();
    System.out.println("Auto selected: " + mAutoSelected);
  }

  @Override
  public void autonomousPeriodic() {
    switch (mAutoSelected) {
      case kCustomAuto:
        break;
      case kDefaultAuto:
      default:
        break;
    }
  }

  @Override
  public void teleopInit() {}

  @Override
  public void teleopPeriodic() {}

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void testInit() {}

  @Override
  public void testPeriodic() {}

  @Override
  public void simulationInit() {}

  @Override
  public void simulationPeriodic() {}
}
