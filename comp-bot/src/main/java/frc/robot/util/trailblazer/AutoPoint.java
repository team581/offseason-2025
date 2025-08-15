package frc.robot.util.trailblazer;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.util.MathHelpers;
import frc.robot.util.trailblazer.constraints.AutoConstraintOptions;
import java.util.Optional;
import java.util.function.Supplier;

// TODO: Convert to record
public class AutoPoint {
  private static Command emptyCommand() {
    return Commands.none().withName("AutoPointEmptyCommand");
  }

  public final Supplier<Pose2d> poseSupplier;
  public final Optional<AutoConstraintOptions> constraints;
  public final Command command;

  public AutoPoint(
      Supplier<Pose2d> poseSupplier, Command command, Optional<AutoConstraintOptions> constraints) {
    this.poseSupplier = poseSupplier;
    this.command = command;
    this.constraints = constraints;
  }

  public AutoPoint(
      Supplier<Pose2d> poseSupplier, Command command, AutoConstraintOptions constraints) {
    this(poseSupplier, command, Optional.of(constraints));
  }

  public AutoPoint(Supplier<Pose2d> poseSupplier, Command command) {
    this(poseSupplier, command, Optional.empty());
  }

  public AutoPoint(Supplier<Pose2d> poseSupplier, AutoConstraintOptions constraints) {
    this(poseSupplier, emptyCommand(), constraints);
  }

  public AutoPoint(Supplier<Pose2d> poseSupplier) {
    this(poseSupplier, emptyCommand(), Optional.empty());
  }

  public AutoPoint(Pose2d pose, Command command, AutoConstraintOptions constraints) {
    this(() -> pose, command, constraints);
  }

  public AutoPoint(Pose2d pose, Command command) {
    this(() -> pose, command);
  }

  public AutoPoint(Pose2d pose, AutoConstraintOptions constraints) {
    this(pose, emptyCommand(), constraints);
  }

  public AutoPoint(Pose2d pose) {
    this(pose, emptyCommand());
  }

  public AutoPoint pathflipped() {
    return new AutoPoint(() -> MathHelpers.pathflip(poseSupplier.get()), command, constraints);
  }
}
