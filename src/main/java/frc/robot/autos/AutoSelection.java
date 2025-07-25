package frc.robot.autos;

import frc.robot.autos.auto_path_commands.blue.BlueDoNothingAuto;
import frc.robot.autos.auto_path_commands.blue.BlueLollipop4L4Auto;
import frc.robot.autos.auto_path_commands.blue.BlueLollipopLeftABAuto;
import frc.robot.autos.auto_path_commands.blue.BlueLollipopRightABAuto;
import frc.robot.autos.auto_path_commands.red.RedDoNothingAuto;
import frc.robot.autos.auto_path_commands.red.RedLollipop4L4Auto;
import frc.robot.autos.auto_path_commands.red.RedLollipopLeftABAuto;
import frc.robot.autos.auto_path_commands.red.RedLollipopRightABAuto;
import frc.robot.robot_manager.RobotManager;
import frc.robot.util.trailblazer.Trailblazer;
import java.util.function.BiFunction;

public enum AutoSelection {
  DO_NOTHING(RedDoNothingAuto::new, BlueDoNothingAuto::new),

  LOLLIPOP_LEFT_FRONT_SIDE(RedLollipopLeftABAuto::new, BlueLollipopLeftABAuto::new),
  LOLLIPOP_RIGHT_FRONT_SIDE(RedLollipopRightABAuto::new, BlueLollipopRightABAuto::new),
  LOLLIPOP_4_L4(RedLollipop4L4Auto::new, BlueLollipop4L4Auto::new);


  public final BiFunction<RobotManager, Trailblazer, BaseAuto> redAuto;
  public final BiFunction<RobotManager, Trailblazer, BaseAuto> blueAuto;

  private AutoSelection(
      BiFunction<RobotManager, Trailblazer, BaseAuto> redAuto,
      BiFunction<RobotManager, Trailblazer, BaseAuto> blueAuto) {
    this.redAuto = redAuto;
    this.blueAuto = blueAuto;
  }
}
