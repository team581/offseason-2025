package com.team581.trailblazer;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj2.command.Subsystem;

public interface SwerveBase extends Subsystem {
  void setFieldRelativeAutoSpeeds(ChassisSpeeds speeds);

  ChassisSpeeds getFieldRelativeSpeeds();
}
