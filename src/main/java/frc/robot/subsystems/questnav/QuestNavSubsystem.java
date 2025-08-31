// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems.questnav;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import gg.questnav.questnav.PoseFrame;
import gg.questnav.questnav.QuestNav;

public class QuestNavSubsystem extends SubsystemBase {

  public QuestNav questNav = new QuestNav();
  private Transform2d QUEST_TO_ROBOT = new Transform2d(Units.inchesToMeters(15.0),0.0,new Rotation2d());
  private SwerveSubsystem swerveSubsystem;

  /** Creates a new QuestNav. */
  public QuestNavSubsystem(SwerveSubsystem swerveSubsystem) {
    this.swerveSubsystem = swerveSubsystem;
    
    // Set intial Position -- Right now, this assumes we're sitting in front of AprilTag 10 on the red side of the field
    questNav.setPose(new Pose2d(Units.inchesToMeters(481.39 - 28),  // Subtract 28" for length of robot
                                Units.inchesToMeters(158.50), 
                                new Rotation2d(Math.toRadians(180))));

  }

  public void updateVisionMeasurement() {
    Matrix<N3, N1> QUESTNAV_STD_DEVS = VecBuilder.fill(0.02,0.02,0.035);

    SmartDashboard.putBoolean("QuestNav.isConnected", questNav.isConnected());
    SmartDashboard.putBoolean("QuestNav.isTracking", questNav.isTracking());
    
    if (questNav.isConnected() && questNav.isTracking()) {

      PoseFrame[] questFrames = questNav.getAllUnreadPoseFrames();
      
      //Loop over the pose data frames nd send them to the pose estimatior

      for (PoseFrame questFrame: questFrames) {
        //Get the Pose of the Quest
        Pose2d questPose = questFrame.questPose();
        
        //get the timestamp for when the data was sent
        double timestamp = questFrame.dataTimestamp();

        // Transform by the mount pose to get the robot pose
        Pose2d robotPose = questPose.transformBy(QUEST_TO_ROBOT.inverse());

        // Add the mesaurement to the pose Estimator
        swerveSubsystem.getSwerveDrive().addVisionMeasurement(robotPose, timestamp,QUESTNAV_STD_DEVS);
        
      }
    }
  }

  public void setQuestNavPose(Pose2d pose) {
    questNav.setPose(pose.transformBy(QUEST_TO_ROBOT));
  }


  @Override
  public void periodic() {
    // This method will be called once per scheduler run

    questNav.commandPeriodic();
    updateVisionMeasurement();
    
  }
}
