// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import static edu.wpi.first.units.Units.DegreesPerSecond;

import java.util.Optional;

import com.studica.frc.AHRS;

import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import limelight.networktables.AngularVelocity3d;
import limelight.networktables.LimelightPoseEstimator;
import limelight.networktables.LimelightSettings;
import limelight.networktables.Orientation3d;
import swervelib.SwerveDrive;
import limelight.networktables.LimelightSettings.LEDMode;
import limelight.Limelight;
import limelight.networktables.PoseEstimate;

public class VisionSubsystem extends SubsystemBase {
  /** Creates a new VisionSubsystem. */

  Limelight limelight;
  LimelightSettings limelightSettings;
  LimelightPoseEstimator poseEstimator;

  Orientation3d robotOrientation3d;

  private boolean locationFoundViaAprilTag = false;

  SwerveDrive swerveDrive;

  public static final Pose3d LIMELIGHT_POSE = new Pose3d(0.0762,
                                                          -0.3236,
                                                          0.5207,
                                                          new Rotation3d(Units.degreesToRadians(0), 
                                                                         Units.degreesToRadians(-14), 
                                                                         Units.degreesToRadians(90)));

  public VisionSubsystem(SwerveDrive swerveDrive) {

    limelight = new Limelight("limelight-front");

    limelight.getSettings().withLimelightLEDMode(LEDMode.PipelineControl)
                           .withCameraOffset(LIMELIGHT_POSE)
                           .withRobotOrientation(new Orientation3d(swerveDrive.getGyro().getRotation3d(), 
                                                                   new AngularVelocity3d(DegreesPerSecond.of(swerveDrive.getGyroRotation3d().getX()), 
                                                                                         DegreesPerSecond.of(swerveDrive.getGyroRotation3d().getY()),
                                                                                         DegreesPerSecond.of(swerveDrive.getGyroRotation3d().getZ()))))
                           .save();

    this.swerveDrive = swerveDrive;
                                                                                

  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    // Check to see if we've already found our location via AprilTag localization. If we have, move on. If not, check 
    // to see if an AprilTag is visible and update location

    if (!getAprilTagLocationSet()) {

      // Get MegaTag2 pose
      Optional<PoseEstimate> visionEstimate = limelight.getPoseEstimator(true).getPoseEstimate();
      // If the pose is present
      visionEstimate.ifPresent((PoseEstimate poseEstimate) -> {
        if (visionEstimate.get().tagCount > 0) {
          // Add it to the pose estimator.
          swerveDrive.addVisionMeasurement(poseEstimate.pose.toPose2d(), poseEstimate.timestampSeconds);
          this.locationFoundViaAprilTag = true;
        }
      });
    }

    SmartDashboard.putBoolean("frc3620/locationFoundViaAprilTag", locationFoundViaAprilTag);
  }

  public boolean getAprilTagLocationSet() {
    return this.locationFoundViaAprilTag;
  }
}
