// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import org.usfirst.frc3620.GitNess;
import org.usfirst.frc3620.RobotMode;
import org.usfirst.frc3620.Utilities;
import org.tinylog.TaggedLogger;
import org.usfirst.frc3620.ChameleonController.ControllerType;
import org.usfirst.frc3620.logger.LoggingMaster;

import dev.doglog.DogLog;
import dev.doglog.DogLogOptions;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;

public class Robot extends TimedRobot {
  private Command m_autonomousCommand;

  private final RobotContainer m_robotContainer;

  private TaggedLogger logger;

  static private RobotMode currentRobotMode = RobotMode.INIT, previousRobotMode;
  
  public Robot() {
    m_robotContainer = new RobotContainer();
  }

  @Override
  public void robotInit() {
    // get data logging going
    DogLog.setOptions(new DogLogOptions().withCaptureDs(true).withCaptureNt(false));
    DataLogManager.start();

    logger = LoggingMaster.getLogger(Robot.class);
    logger.info ("THIS IS GREG'S TEST MESSAGE");
    logger.info ("I'm Alive! {}", GitNess.gitDescription());
    Utilities.logMetadataToDataLog();

    Utilities.addDataLogForNT("frc3620");
    Utilities.addDataLogForNT("SmartDashboard/frc3620");

    StringBuilder sb = new StringBuilder();
    sb.append(GitNess.getCommitId());
    Boolean dirty = GitNess.getDirty();

    if (dirty == null) {
      sb.append("-unknownDirtyness");
    } else {
      sb.append(dirty ? "-dirty" : "");
    }

    SmartDashboard.putString("frc3620/git.commit.id",sb.toString());

  }
  @Override
  public void robotPeriodic() {
    CommandScheduler.getInstance().run();
  }

  @Override
  public void disabledInit() {}

  @Override
  public void disabledPeriodic() {}

  @Override
  public void disabledExit() {}

  @Override
  public void autonomousInit() {
    m_autonomousCommand = m_robotContainer.getAutonomousCommand();

    if (m_autonomousCommand != null) {
      m_autonomousCommand.schedule();
    }
  }

  @Override
  public void autonomousPeriodic() {}

  @Override
  public void autonomousExit() {}

  @Override
  public void teleopInit() {
    if (m_autonomousCommand != null) {
      m_autonomousCommand.cancel();
    }

    setupDriverController();
    
  }

  @Override
  public void teleopPeriodic() {}

  @Override
  public void teleopExit() {}

  @Override
  public void testInit() {
    CommandScheduler.getInstance().cancelAll();

  }

  @Override
  public void testPeriodic() {}

  @Override
  public void testExit() {}

  void setupDriverController() {
    String driveControllerName = m_robotContainer.getDriverControllerName();
    logger.info("Drive Controler Name: '{}'", driveControllerName);
    if (driveControllerName.startsWith("FlySky")) {
      m_robotContainer.setDriverContollerName(ControllerType.B);
    } else {
      m_robotContainer.setDriverContollerName(ControllerType.A);
    }

  }
}
