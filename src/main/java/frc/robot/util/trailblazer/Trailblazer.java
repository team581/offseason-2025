package frc.robot.util.trailblazer;

import dev.doglog.DogLog;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.autos.TrailblazerPathLogger;
import frc.robot.localization.LocalizationSubsystem;
import frc.robot.swerve.SwerveSubsystem;
import frc.robot.util.trailblazer.followers.PathFollower;
import frc.robot.util.trailblazer.followers.PidPathFollower;
import frc.robot.util.trailblazer.trackers.PathTracker;
import frc.robot.util.trailblazer.trackers.pure_pursuit.PurePursuitPathTracker;

public class Trailblazer {
  private final SwerveSubsystem swerve;
  private final LocalizationSubsystem localization;
  private final PathTracker pathTracker = new PurePursuitPathTracker();
  private final PathFollower pathFollower =
      new PidPathFollower(new PIDController(3.5, 0, 0), new PIDController(7, 0, 0));
  private int previousAutoPointIndex = -1;

  public Trailblazer(SwerveSubsystem swerve, LocalizationSubsystem localization) {
    this.swerve = swerve;
    this.localization = localization;
  }

  public Command followSegment(AutoSegment segment) {
    return followSegment(segment, true);
  }

  public Command followSegment(AutoSegment segment, boolean shouldEnd) {
    TrailblazerPathLogger.logSegment(segment);
    var command =
        Commands.runOnce(
                () -> {
                  pathTracker.resetAndSetPoints(segment.points);
                  previousAutoPointIndex = -1;
                  DogLog.log(
                      "Autos/Trailblazer/CurrentSegment/InitialPoints",
                      segment.points.stream()
                          .map(point -> point.poseSupplier.get())
                          .toArray(Pose2d[]::new));
                })
            .alongWith(
                Commands.run(
                    () -> {
                      pathTracker.updateRobotState(
                          localization.getPose(), swerve.getFieldRelativeSpeeds());
                      var currentAutoPointIndex = pathTracker.getCurrentPointIndex();
                      var currentAutoPoint = segment.points.get(currentAutoPointIndex);

                      var constrainedVelocityGoal = getSwerveSetpoint(currentAutoPoint, segment);
                      swerve.setFieldRelativeAutoSpeeds(constrainedVelocityGoal);

                      DogLog.log("Autos/Trailblazer/Tracker/Output", pathTracker.getTargetPose());
                      DogLog.log("Autos/Trailblazer/Follower/Output", constrainedVelocityGoal);

                      DogLog.log(
                          "Autos/Trailblazer/UsedConstraints/MaxLinearV",
                          segment.getConstraints(currentAutoPoint).linearConstraints().maxVelocity);
                      DogLog.log(
                          "Autos/Trailblazer/UsedConstraints/MaxLinearA",
                          segment
                              .getConstraints(currentAutoPoint)
                              .linearConstraints()
                              .maxAcceleration);
                      DogLog.log(
                          "Autos/Trailblazer/UsedConstraints/MaxAngularV",
                          segment
                              .getConstraints(currentAutoPoint)
                              .angularConstraints()
                              .maxVelocity);
                      DogLog.log(
                          "Autos/Trailblazer/UsedConstraints/MaxAngularA",
                          segment
                              .getConstraints(currentAutoPoint)
                              .angularConstraints()
                              .maxAcceleration);

                      DogLog.log(
                          "Autos/Trailblazer/Tracker/CurrentPointIndex", currentAutoPointIndex);
                      if (previousAutoPointIndex != currentAutoPointIndex) {
                        // Currently tracked point has changed, trigger side effects

                        // Each of the points in (previous, current]
                        var pointsToRunSideEffectsFor =
                            segment.points.subList(
                                previousAutoPointIndex + 1, currentAutoPointIndex + 1);
                        for (var passedPoint : pointsToRunSideEffectsFor) {
                          DogLog.log(
                              "Autos/Trailblazer/Tracker/CommandTriggered",
                              passedPoint.command.getName());
                          passedPoint.command.schedule();
                        }
                        previousAutoPointIndex = currentAutoPointIndex;
                      }
                    },
                    swerve))
            .withName("FollowSegmentIndefinitely");

    if (shouldEnd) {
      return command
          .until(
              () -> segment.isFinished(localization.getPose(), pathTracker.getCurrentPointIndex()))
          .andThen(Commands.runOnce(() -> swerve.setFieldRelativeAutoSpeeds(new ChassisSpeeds())))
          .withName("FollowSegmentUntilFinished");
    }

    return command;
  }

  private ChassisSpeeds getSwerveSetpoint(AutoPoint point, AutoSegment segment) {
    var robotPose = localization.getPose();
    var goalPose = pathTracker.getTargetPose();
    var distanceToSegmentEnd =
        segment.getRemainingDistance(localization.getPose(), pathTracker.getCurrentPointIndex());
    var constraints = segment.getConstraints(point);

    var currentSpeeds = swerve.getFieldRelativeSpeeds();
    var decelerationDistance =
        (currentSpeeds.vMetersPerSecond * currentSpeeds.vMetersPerSecond)
            / (2.0 * constraints.linearConstraints().maxAcceleration);
    var perfectVelocity =
        Math.sqrt(2.0 * (constraints.linearConstraints().maxAcceleration * distanceToSegmentEnd));

    var speedsForCurrentPoint =
        pathFollower.calculateSpeeds(robotPose, goalPose, currentSpeeds, constraints);

    if (distanceToSegmentEnd < decelerationDistance) {
      // Need to start decelerating
      speedsForCurrentPoint.vMetersPerSecond =
          Math.min(perfectVelocity, speedsForCurrentPoint.vMetersPerSecond);
    }

    DogLog.log("Autos/Trailblazer/Follower/OutputLinV", speedsForCurrentPoint.vMetersPerSecond);
    DogLog.log(
        "Autos/Trailblazer/Follower/Decelerating",
        speedsForCurrentPoint.vMetersPerSecond == perfectVelocity);

    return speedsForCurrentPoint;
  }
}
