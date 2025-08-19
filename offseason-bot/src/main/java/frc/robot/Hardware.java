package frc.robot;

import edu.wpi.first.wpilibj2.command.button.CommandXboxController;

public class Hardware {
  // public final PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);
  public final CommandXboxController driverController = new CommandXboxController(0);
  public final CommandXboxController operatorController = new CommandXboxController(1);
}
