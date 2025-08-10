// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import org.opencv.core.Mat;
import org.tinylog.TaggedLogger;
import org.usfirst.frc3620.CANDeviceFinder;
import org.usfirst.frc3620.CANDeviceType;
import org.usfirst.frc3620.ChameleonController;
import org.usfirst.frc3620.FlySkyConstants;
import org.usfirst.frc3620.RobotParametersBase;
import org.usfirst.frc3620.RobotParametersContainer;
import org.usfirst.frc3620.Utilities;
import org.usfirst.frc3620.XBoxConstants;
import org.usfirst.frc3620.ChameleonController.ControllerType;
import org.usfirst.frc3620.logger.LoggingMaster;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc.robot.Constants.OperatorConstants;
import frc.robot.subsystems.swervedrive.SwerveSubsystem;
import swervelib.SwerveInputStream;
import swervelib.parser.json.modules.DriveConversionFactorsJson;


public class RobotContainer {

  // Setup Logging
  public final static TaggedLogger logger = LoggingMaster.getLogger(RobotContainer.class);

  // Define Joysticks
  public static ChameleonController driverJoystick;

  // Standard Average Utilities
  public static CANDeviceFinder canDeviceFinder;
  public static RobotParameters robotParameters;

  // Subsystems
  public static SwerveSubsystem swerveSubsystem;

  public RobotContainer() {

    canDeviceFinder = new CANDeviceFinder();
    robotParameters = RobotParametersContainer.getRobotParameters(RobotParameters.class);
    logger.info("Got Parameters for chassis '{}'", robotParameters.getName());


    boolean iAmACompetitionRobot = amIACompBot();
    if (!iAmACompetitionRobot) {
      logger.warn("this is a test chassis. Will try to deal with missing hardware.");
    }



    configureBindings();

    makeSubsystems();

    configureSwerveDrive();
  
    DriverStation.silenceJoystickConnectionWarning(true);

    Utilities.addDataLogForNT("SmartDashboard/swerve");

  }

  private void configureBindings() {

    driverJoystick = new ChameleonController(new Joystick(0));

    //TODO: NavX Reset
    
  }


  public Command getAutonomousCommand() {
    return Commands.print("No autonomous command configured");
  }

  private void makeSubsystems() {
    
    // Setup Swerve Drive 
    if (canDeviceFinder.isDevicePresent(CANDeviceType.TALON_PHOENIX6, 1, "Swerve Drive 1") || 
        shouldMakeAllCANDevices() || Robot.isSimulation()) {

      canDeviceFinder.isDevicePresent(CANDeviceType.CANCODER_PHOENIX6, 3, "Swerve Drive 3"); 
      canDeviceFinder.isDevicePresent(CANDeviceType.CANCODER_PHOENIX6, 5, "Swerve Drive 5");
      canDeviceFinder.isDevicePresent(CANDeviceType.CANCODER_PHOENIX6, 7, "Swerve Drive 7");
      canDeviceFinder.isDevicePresent(CANDeviceType.SPARK_MAX,2,"Swerve Drive 2");
      canDeviceFinder.isDevicePresent(CANDeviceType.SPARK_MAX,4,"Swerve Drive 4");
      canDeviceFinder.isDevicePresent(CANDeviceType.SPARK_MAX,6,"Swerve Drive 6");
      canDeviceFinder.isDevicePresent(CANDeviceType.SPARK_MAX,8,"Swerve Drive 8");
      
      String swerveFolder = robotParameters.getSwerveDirectoryName();
      if (swerveFolder == null) swerveFolder = "swerve/simulation";

      SmartDashboard.putString("frc3620/swerveFolder", swerveFolder);
      logger.info("Using swerveFolder '{}'", swerveFolder);

      swerveSubsystem = new SwerveSubsystem(new File(Filesystem.getDeployDirectory(), swerveFolder));
      SmartDashboard.putData("frc3620/swerveSubsystem", swerveSubsystem);

      }
    
    // TODO: Integrate Health Subsystem



  }

  public String getDriverControllerName() {
    return driverJoystick.getName();
  }

  public void setDriverContollerName(ControllerType driveControllerType) {
    driverJoystick.setCurrentControllerType(driveControllerType);
  }


  /**
   * Determine if this is a Competition Robot
   * 
   * If it is connected to an FMS
   * 
   * if it is missing a grounding Jumper
   * 
   * if the robotparameters.json says so.
   * 
   * @return
   */

  @SuppressWarnings({ "unused", "RedundantIfStatement", "PointlessBooleanExpression" })
  public static boolean amIACompBot() {

    if(DriverStation.isFMSAttached()) {
      return true;
    }

    if(robotParameters.isCompetitionRobot()) {
      return true;
    }

    return false;

  }

  /**
   * Determine if we should make software objects, even if the device does not appear
   * on the CAN bus.
   * 
   * We should if it's connected to an FMS
   * 
   * We should if it is missing a grounding juper on DIO0
   * 
   * WE should if robot_parameters.json says so
   * 
   */

   @SuppressWarnings({ "unused", "RedundantIfStatement" })
   public static boolean shouldMakeAllCANDevices() {
    if (amIACompBot()) {
      return true;
    }

    if (robotParameters.shouldMakeAllCANDevices()) {
      return true;
    }

    return false;

   }


   public static double getDriveVerticalJoystick() {
    double axisValue = driverJoystick.getRawAxis(XBoxConstants.AXIS_LEFT_Y, FlySkyConstants.AXIS_LEFT_Y);
    double deadband = 0.1;
    if (driverJoystick.getCurrentControllerType() == ControllerType.B) {
      deadband = 0.02;
    }
    SmartDashboard.putNumber("frc3620/driver.y.raw", axisValue);
    axisValue = MathUtil.applyDeadband(axisValue, deadband);
    return axisValue;

   }

   public static double getDriveHorizontalJoystick() {
    double axisValue = driverJoystick.getRawAxis(XBoxConstants.AXIS_LEFT_X, FlySkyConstants.AXIS_LEFT_X);
    double deadband = 0.05;
    if (driverJoystick.getCurrentControllerType() == ControllerType.B) {
      deadband = 0.02;
    }
    SmartDashboard.putNumber("frc3620/driver.x.raw", axisValue);
    axisValue = MathUtil.applyDeadband(axisValue, deadband);
    return axisValue * axisValue * Math.signum(axisValue);

   }

   public static double getDriveSpinJoystick() {
    double axisValue = driverJoystick.getRawAxis(XBoxConstants.AXIS_RIGHT_X, FlySkyConstants.AXIS_RIGHT_X);
    double deadband = 0.05;
    if (driverJoystick.getCurrentControllerType() == ControllerType.B) {
      deadband = 0.02;
    }
    SmartDashboard.putNumber("frc3620/driver.spin.raw", axisValue);
    axisValue = MathUtil.applyDeadband(axisValue, deadband);
    return axisValue * axisValue * Math.signum(axisValue);
   }

   public static void configureSwerveDrive() {

    if (swerveSubsystem != null) {

      SwerveInputStream driveAngularVelocityStream = SwerveInputStream.of(swerveSubsystem.getSwerveDrive(),
          () -> getDriveVerticalJoystick() * -1,
          () -> getDriveHorizontalJoystick() * -1) 
          .withControllerRotationAxis(() -> getDriveSpinJoystick() * -1)
          .deadband(OperatorConstants.DEADBAND)
          .scaleTranslation(1.0)
          .allianceRelativeControl(true);

      SwerveInputStream driveRobotOrientedStream = driveAngularVelocityStream.copy()
          .robotRelative(true)
          .allianceRelativeControl(false);
      
      SwerveInputStream driveRobotOrientedSlowStream = driveRobotOrientedStream.copy()
          .scaleTranslation(0.3);
          
      Command driveFieldOrientedAngularVelocityCommand = swerveSubsystem.driveFieldOriented(driveAngularVelocityStream)
          .withName("DriveFromJoystick");

      Command driveRobotOrientedSlowCommand = swerveSubsystem.driveFieldOriented(driveRobotOrientedSlowStream)
          .withName("DriveFromJoystickSlow");

      // Set Swerve Default Command
      if (RobotBase.isSimulation()) {
        swerveSubsystem.setDefaultCommand(driveFieldOrientedAngularVelocityCommand);
      } else {
        swerveSubsystem.setDefaultCommand(driveFieldOrientedAngularVelocityCommand);
      }
    }
   }

}
