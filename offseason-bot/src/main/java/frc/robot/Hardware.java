package frc.robot;

import com.ctre.phoenix6.hardware.CANdi;
import com.ctre.phoenix6.hardware.TalonFX;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.config.RobotConfig;

public class Hardware {
  // public final PowerDistribution pdh = new PowerDistribution(1, ModuleType.kRev);
  public final CommandXboxController driverController = new CommandXboxController(0);
  public final CommandXboxController operatorController = new CommandXboxController(1);

  public final TalonFX intakeMotor = new TalonFX(RobotConfig.get().intake().motorId(), RobotConfig.get().intake().canBusName());

  public final TalonFX deployMotor = new TalonFX(RobotConfig.get().deploy().motorId(), RobotConfig.get().deploy().canBusName());

  public final TalonFX leftSingulatorMotor = new TalonFX(RobotConfig.get().singulator().leftMotorId(), RobotConfig.get().singulator().canBusName());
  public final TalonFX rightSingulatorMotor = new TalonFX(RobotConfig.get().singulator().rightMotorId(), RobotConfig.get().singulator().canBusName());

  public final CANdi intakeTopCANdi = new CANdi(RobotConfig.get().intake().topCANdiId());
  public final CANdi intakeBottomCANdi = new CANdi(RobotConfig.get().intake().bottomCANdiId());
}
