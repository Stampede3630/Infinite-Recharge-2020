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
<<<<<<< Updated upstream

  private Drivetrain m_swerve;
  private Compressor compp = new Compressor(0);
  @Override
  public void robotInit() {
=======
  private Compressor comp = new Compressor(0);
  private Climber climber;

  private IntakeIndex ballProcessor;
  private Shooter shoot;
  private BreakBeam breakBeam;
  private Drivetrain m_swerve; 
  private Chooser chooser;
  private TrajectoryContainer tContainer;
  

  @Override
  public void robotInit() {
    m_swerve = Drivetrain.getInstance();
    chooser = new Chooser();
    shoot = new Shooter();
    ballProcessor = new IntakeIndex();
    climber = new Climber();
    tContainer = new TrajectoryContainer();
    SmartDashboard.putNumber("kP", 1);
    SmartDashboard.putNumber("kF", 0.055);
    SmartDashboard.putNumber("kI", 0);
    SmartDashboard.putNumber("kD", 0);
    breakBeam = BreakBeam.getInstance();
    //BallFollowDrive.resetIntakeState();
>>>>>>> Stashed changes

    m_swerve = new Drivetrain();
   
  }

  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Current trolleySpark", RobotMap.trolleySpark.getOutputCurrent());
    SmartDashboard.putNumber("Current Elevator", RobotMap.elevatorSpark.getOutputCurrent());
  }

  @Override
  public void autonomousInit() {
    RobotMap.resetEncoders();

  }

  @Override
  public void autonomousPeriodic() {
<<<<<<< Updated upstream
=======
    tContainer.trajectoryFollowing.auto();
    m_swerve.updateOdometry();
>>>>>>> Stashed changes

  }

  @Override
  public void teleopPeriodic() {
<<<<<<< Updated upstream
    //m_swerve.driveWithJoystick(true);
    RobotMap.elevatorSpark.set(RobotMap.controller.getX(Hand.kRight) *-.8);
    RobotMap.trolleySpark.set(RobotMap.controller.getY(Hand.kRight) *-.5);
    System.out.println(RobotMap.controller.getY(Hand.kRight) *.5 + " , " + RobotMap.controller.getX(Hand.kRight) *.5);
=======
    m_swerve.driveWithJoystick(true);
    // Systemtrue.out.println(RobotMap.controller.getY(Hand.kRight) *.5 + " , " +
    // RobotMap.controller.getX(Hand.kRight) *.5);
    //shoot.control();
    //ballProcessor.manualControl();
    //ballProcessor.ToggleSolenoids();
    climber.climberPeriodic();
    //ballProcessor.index();
    shoot.control();
    ballProcessor.ToggleSolenoids();


>>>>>>> Stashed changes


  }

  @Override
  public void testPeriodic() {

  }

  
}
