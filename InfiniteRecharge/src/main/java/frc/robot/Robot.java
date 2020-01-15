/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
  /**
   * This function is run when the robot is first started up and should be used
   * for any initialization code.
   */
  public static final double kMaxSpeed = 1; // 3 meters per second
  public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second
  public static XboxController m_controller;
  private Drivetrain m_swerve; 
  @Override
  public void robotInit() {
    m_swerve = new Drivetrain();
    m_controller = new XboxController(0);
  }

  @Override
  public void robotPeriodic() {
    m_swerve.postToSmartDashboard();
  }

  @Override
  public void autonomousInit() {
  }
  
  @Override
  public void autonomousPeriodic() {
  }

  @Override
  public void teleopInit() {
  }

  @Override
  public void teleopPeriodic() {
    driveWithJoystick(false);
    
  }

  @Override
  public void testInit() {
  }

  @Override
  public void testPeriodic() {
  }

  private void driveWithJoystick(boolean fieldRelative) {
    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    var xSpeed = -m_controller.getY(Hand.kLeft)* kMaxSpeed;
    if(Math.abs(xSpeed) < (0.2 * kMaxSpeed))
    {
      xSpeed = 0;
    }
    
    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    var ySpeed = -m_controller.getX(Hand.kLeft) * kMaxSpeed;
    if(Math.abs(ySpeed) < (0.2 * kMaxSpeed))
    {
      ySpeed = 0;
    }
    // Get the rate of angular rotation. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    var rot = m_controller.getX(Hand.kRight) * kMaxAngularSpeed;
    if(Math.abs(rot) < (0.2 * kMaxAngularSpeed))
    {
      rot = 0;
    }
    //System.out.println("rot: " + m_controller.getX(Hand.kRight));
    //System.out.println("rot-c: " + rot);
    System.out.println(xSpeed + "," + ySpeed);
    m_swerve.drive(xSpeed, ySpeed, rot, fieldRelative);
  }
}