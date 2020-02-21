/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  private RumbleSequence imperialRumble = new RumbleSequence(RumbleSequence.Sequences.IMPERIAL_RUMBLE);
  private Climber climber;
  private IntakeIndex ballProcessor;
  private Shooter shoot;
  private BreakBeam breakBeam;
  private Drivetrain m_swerve; 
  private Chooser chooser;
  private TrajectoryContainer tContainer;
  private ServoMotor servoMotor;
  private Compressor comp = new Compressor(0);
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
    servoMotor = new ServoMotor();
  }

  @Override
  public void robotPeriodic() {
    //SmartDashboard.putNumber("Current trolleySpark", RobotMap.ClimberMap.TROLLEY_SPARK.getOutputCurrent());
    //SmartDashboard.putNumber("Current Elevator", RobotMap.ClimberMap.ELEVATOR_SPARK.getOutputCurrent());
    ballProcessor.toSmartDashboard();
    ballProcessor.updateBooleans();
    shoot.smartDashboardOutput();
    m_swerve.postToSmartDashboard();

    RumbleSystem.update(); // Handles rumbling - DON'T remove this, otherwise rumble feedback stops working
 
    if (RobotMap.CONTROLLER.getBumperPressed(Hand.kRight)){ // TODO: Remove this if not needed
      imperialRumble.trigger();
    }
    if (RobotMap.CONTROLLER.getBumperPressed(Hand.kLeft)){
      imperialRumble.reset();
    }

    breakBeam.toSmartDashBoard();
  }

  @Override
  public void autonomousInit() {
    RobotMap.resetEncoders();
    tContainer.trajectoryFollowing.restAll();

  }

  @Override
  public void autonomousPeriodic() {
    tContainer.trajectoryFollowing.auto();
    m_swerve.updateOdometry();
    System.out.println("Total Time Seconds"+tContainer.trajectoryFollowing.trajectory.getTotalTimeSeconds());
    System.out.println("Total Time Seconds Robot"+tContainer.trajectoryFollowing.m_timer.get());
  }

  @Override
  public void teleopPeriodic() {
    m_swerve.driveWithJoystick(true);
    servoMotor.ServoUp();
    climber.climberPeriodic();
    ballProcessor.index();
    shoot.control();
    ballProcessor.ToggleSolenoids();
    


  }

  @Override
  public void testInit() {
    // Sets up the limelight pipeline
    //BallFollowDrive.initLimelight();
  }

  @Override
  public void testPeriodic() {
    /*
    // Button A on the XBOX makes the robot start searching again (if it marked
    // intake as done)
    if (RobotMap.CONTROLLER.getAButton()) {
      BallFollowDrive.resetIntakeState();
    }

    // Intake code runs only while the right bumber is held (otherwise it stops)
    if (RobotMap.CONTROLLER.getBumper(Hand.kRight)) {
      BallFollowDrive.drive();
    } else {
      BallFollowDrive.stop();
    }
  */
    //Drivetrain.postToSmartDashboard();


  }

}
