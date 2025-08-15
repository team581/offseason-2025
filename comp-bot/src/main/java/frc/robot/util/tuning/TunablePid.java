package frc.robot.util.tuning;

import com.ctre.phoenix6.configs.TalonFXConfiguration;
import com.ctre.phoenix6.hardware.TalonFX;
import dev.doglog.DogLog;
import frc.robot.config.RobotConfig;

public class TunablePid {
  public static void of(String key, TalonFX motor, TalonFXConfiguration defaultConfig) {
    if (!RobotConfig.IS_DEVELOPMENT) {
      return;
    }

    DogLog.tunable(
        key + "/kP",
        defaultConfig.Slot0.kP,
        newP -> motor.getConfigurator().apply(defaultConfig.Slot0.withKP(newP)));
    DogLog.tunable(
        key + "/kI",
        defaultConfig.Slot0.kI,
        newI -> motor.getConfigurator().apply(defaultConfig.Slot0.withKI(newI)));
    DogLog.tunable(
        key + "/kD",
        defaultConfig.Slot0.kD,
        newD -> motor.getConfigurator().apply(defaultConfig.Slot0.withKD(newD)));
    DogLog.tunable(
        key + "/kS",
        defaultConfig.Slot0.kS,
        newS -> motor.getConfigurator().apply(defaultConfig.Slot0.withKS(newS)));
    DogLog.tunable(
        key + "/kV",
        defaultConfig.Slot0.kV,
        newV -> motor.getConfigurator().apply(defaultConfig.Slot0.withKV(newV)));
    DogLog.tunable(
        key + "/kA",
        defaultConfig.Slot0.kA,
        newA -> motor.getConfigurator().apply(defaultConfig.Slot0.withKA(newA)));
    DogLog.tunable(
        key + "/kG",
        defaultConfig.Slot0.kG,
        newG -> motor.getConfigurator().apply(defaultConfig.Slot0.withKG(newG)));

    DogLog.tunable(
        key + "/MotionMagic/MaxVel",
        defaultConfig.MotionMagic.MotionMagicCruiseVelocity,
        newMaxVelocity ->
            motor
                .getConfigurator()
                .apply(defaultConfig.MotionMagic.withMotionMagicCruiseVelocity(newMaxVelocity)));
    DogLog.tunable(
        key + "/MotionMagic/MaxAccel",
        defaultConfig.MotionMagic.MotionMagicCruiseVelocity,
        newMaxAccel ->
            motor
                .getConfigurator()
                .apply(defaultConfig.MotionMagic.withMotionMagicAcceleration(newMaxAccel)));
  }

  private TunablePid() {}
}
