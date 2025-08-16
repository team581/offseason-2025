package frc.robot.autos;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.robot_manager.RobotManager;
import frc.robot.util.trailblazer.Trailblazer;

public class AutoChooser {
  private final SendableChooser<AutoSelection> chooser = new SendableChooser<>();

  public AutoChooser(RobotManager robotManager, Trailblazer trailblazer) {
    SmartDashboard.putData("Autos/SelectedAuto", chooser);

    for (AutoSelection selection : AutoSelection.values()) {
      chooser.addOption(selection.toString(), selection);
    }

    chooser.setDefaultOption(AutoSelection.DO_NOTHING.toString(), AutoSelection.DO_NOTHING);
  }

  public AutoSelection getSelectedAuto() {
    return chooser.getSelected();
  }
}
