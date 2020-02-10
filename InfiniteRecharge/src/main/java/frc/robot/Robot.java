/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

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

  private IntakeIndex ballProcessor;
  private Shooter shoot;

  @Override
  public void robotInit() {

    m_swerve = Drivetrain.getInstance();
    shoot = new Shooter();
    ballProcessor = new IntakeIndex();
    SmartDashboard.putNumber("kP", 0);
    SmartDashboard.putNumber("kF", 0);
    SmartDashboard.putNumber("kI", 0);
    SmartDashboard.putNumber("kD", 0);

  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Current trolleySpark", RobotMap.trolleySpark.getOutputCurrent());
    SmartDashboard.putNumber("Current Elevator", RobotMap.elevatorSpark.getOutputCurrent());
    ballProcessor.toSmartDashboard();
    ballProcessor.updateBooleans();
    shoot.smartDashboardOutput();
    

  }

  @Override
  public void autonomousInit() {

  }

  @Override
  public void autonomousPeriodic() {

  }

  @Override
  public void teleopPeriodic() {
    //m_swerve.driveWithJoystick(true);
    //RobotMap.elevatorSpark.set(RobotMap.controller.getX(Hand.kRight) *-.8);
    //RobotMap.trolleySpark.set(RobotMap.controller.getY(Hand.kRight) *-.5);
    //System.out.println(RobotMap.controller.getY(Hand.kRight) *.5 + " , " + RobotMap.controller.getX(Hand.kRight) *.5);
    shoot.control();
    ballProcessor.manualControl();
    ballProcessor.ToggleSolenoids();
  

  }

  @Override
  public void testPeriodic() {

  }

  
}
