package frc.robot.config;

import com.ctre.phoenix6.configs.CurrentLimitsConfigs;
import com.ctre.phoenix6.configs.MotorOutputConfigs;
import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import edu.wpi.first.math.filter.Debouncer;
import edu.wpi.first.math.filter.Debouncer.DebounceType;
import frc.robot.config.RobotConfig.DeployConfig;
import frc.robot.config.RobotConfig.IntakeConfig;
import frc.robot.config.RobotConfig.SingulatorConfig;
import frc.robot.config.RobotConfig.SwerveConfig;
import frc.robot.generated.CompBotTunerConstants;

class CompConfig {
  private static final String CANIVORE_NAME = CompBotTunerConstants.kCANBus.getName();
  private static final String RIO_CAN_NAME = "rio";

  public static final RobotConfig competitionBot =
      new RobotConfig(
          "comp",
          new IntakeConfig(
              CANIVORE_NAME,
              99,
              99,
              99,
              new Debouncer(0.3, DebounceType.kBoth),
              new Debouncer(0.3, DebounceType.kBoth),
              new TalonFXConfiguration()
                  .withCurrentLimits(
                      new CurrentLimitsConfigs()
                          .withStatorCurrentLimit(150)
                          .withSupplyCurrentLimit(150))
                  .withMotorOutput(
                      new MotorOutputConfigs()
                          .withInverted(InvertedValue.Clockwise_Positive)
                          .withNeutralMode(NeutralModeValue.Coast))),
          new DeployConfig(
              CANIVORE_NAME,
              99,
              99,
              0.0,
              0.0,
              0.0,
              new TalonFXConfiguration()
                  .withCurrentLimits(
                      new CurrentLimitsConfigs()
                          .withStatorCurrentLimit(150)
                          .withSupplyCurrentLimit(150))
                  .withMotorOutput(
                      new MotorOutputConfigs()
                          .withInverted(InvertedValue.Clockwise_Positive)
                          .withNeutralMode(NeutralModeValue.Coast))),
          new SingulatorConfig(
              CANIVORE_NAME, 99, 99, 99, new TalonFXConfiguration(), new TalonFXConfiguration()),
          new SwerveConfig(new PhoenixPIDController(5.75, 0, 0), true, true, true));

  private CompConfig() {}
}
