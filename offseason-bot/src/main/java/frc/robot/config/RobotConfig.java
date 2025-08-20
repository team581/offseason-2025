package frc.robot.config;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.swerve.utility.PhoenixPIDController;
import edu.wpi.first.math.filter.Debouncer;

public record RobotConfig(
    String robotName,
    IntakeConfig intake,
    DeployConfig deploy,
    SingulatorConfig singulator,
    SwerveConfig swerve) {

  public record IntakeConfig(
      String canBusName,
      int motorId,
      int topCaNdiId,
      int bottomCaNdiId,
      Debouncer topDebouncer,
      Debouncer bottomDebouncer,
      TalonFXConfiguration motorConfig) {}

  public record SingulatorConfig(
      String canBusName,
      int leftMotorId,
      int rightMotorId,
      int candiId,
      TalonFXConfiguration leftMotorConfig,
      TalonFXConfiguration rightMotorConfig) {}

  public record DeployConfig(
      String canBusName,
      int motorId,
      int candiId,
      double homingVoltage,
      double homingCurrentThreshold,
      double homingEndPosition,
      TalonFXConfiguration motorConfig) {}

  public record SwerveConfig(
      PhoenixPIDController snapController,
      boolean invertRotation,
      boolean invertX,
      boolean invertY) {}

  public static final String SERIAL_NUMBER = System.getenv("serialnum");

  public static RobotConfig get() {
    return CompConfig.competitionBot;
  }
}
