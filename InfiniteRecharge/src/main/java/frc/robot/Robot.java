/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private Drivetrain m_swerve;
  private Compressor compp = new Compressor(0);
  @Override
  public void robotInit() {

    m_swerve = new Drivetrain();
  }
  
  @Override
  public void robotPeriodic() {
    //SmartDashboard.putNumber("Current trolleySpark", RobotMap.trolleySpark.getOutputCurrent());
    //SmartDashboard.putNumber("Current Elevator", RobotMap.elevatorSpark.getOutputCurrent());
    m_swerve.postToSmartDashboard();
    m_swerve.updateOdometry();
    shooter.smartDashboardOutput();
  }

  @Override
  public void autonomousInit() {
  }

  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void teleopPeriodic() {
    
    m_swerve.driveWithJoystick(false);
    
    /*
    RobotMap.elevatorSpark.set(RobotMap.controller.getX(Hand.kRight) *-.8);
    RobotMap.trolleySpark.set(RobotMap.controller.getY(Hand.kRight) *-.5);
    System.out.println(RobotMap.controller.getY(Hand.kRight) *.5 + " , " + RobotMap.controller.getX(Hand.kRight) *.5);

    driveWithJoystick(true);
    manipTest.periodic();
    if(m_controller.getTriggerAxis(Hand.kLeft)>0.5)
    {
      shooter.control();
    }
    else
    {
      shooter.drive();
    }
    
*/
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

  
}

