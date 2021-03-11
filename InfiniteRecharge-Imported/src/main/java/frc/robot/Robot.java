/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrajectoryParameterizer;
import frc.robot.RobotMap.DrivetrainMap;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;


/**
 * The VM is configured to automatically run this class, and to call the
 * functions corresponding to each mode, as described in the TimedRobot
 * documentation. If you change the name of this class or the package after
 * creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {
Timer timmy;
PIDController piddy;
public static final int PID_LOOP_IDX = 0;// this is the only pid loop i could find

//private RumbleSequence imperialRumble = new RumbleSequence(RumbleSequence.Sequences.IMPERIAL_RUMBLE);
private BasicAuto basicAuto = new BasicAuto();
private boolean debugging = false;


	// private Compressor comp = new Compressor(0);

	@Override
	public void robotInit() {

		SmartDashboard.putNumber("RPMEdit", 0);
		SmartDashboard.putBoolean("debugging", true);
		TrajectoryContainer.getInstance().trajectoryFollowing.resetAll();
		RobotMap.resetEncoders();
		// BallFollowDrive.resetIntakeState();

	}

	@Override
	public void robotPeriodic() {
		
		if(SmartDashboard.getBoolean("debugging", false))
		{
			Shooter.getInstance().smartDashboardOutput();
			Drivetrain.getInstance().postToSmartDashboard();
			
			ServoMotor.getInstance().setServoSmartDashboard();
			SmartDashboard.putNumber("Odometry X", RobotMap.DrivetrainMap.ODOMETRY.getPoseMeters().getTranslation().getX());
			SmartDashboard.putNumber("Odometry Y", RobotMap.DrivetrainMap.ODOMETRY.getPoseMeters().getTranslation().getY());
		}
		
		IntakeIndex.getInstance().updateBooleans();
		ServoMotor.getInstance().servoPeriodic();
		Chooser.getInstance().chooserPeriodic();
		Limelight.limelightPeriodic();
		SmartDashboard.putNumber("Navx REAL", -RobotMap.SensorMap.GYRO.getYaw());
		RobotMap.StateChooser.RPM  = RobotMap.StateChooser.RPM + SmartDashboard.getNumber("RPMEdit", 0);
		SmartDashboard.putNumber("RPM", -Shooter.getRPM());
		Drivetrain.getInstance().updateOdometry();

		//INDEX DEBUGGING 1/20/2021
		IntakeIndex.getInstance().showButtons();
		//Climber.getInstance().climberDebug();	
		SmartDashboard.putNumber("travectory time", TrajectoryContainer.getInstance().trajectoryFollowing.trajectory.getTotalTimeSeconds());

		SmartDashboard.putNumber("Y value",RobotMap.CONTROLLER.getY(Hand.kLeft));
	}

	@Override
	public void autonomousInit() {
		
		RobotMap.setDriveTalonsBrake();
		RobotMap.resetEncoders();
		//TrajectoryContainer.getInstance().trajectoryFollowing.resetAll();
		basicAuto.resetAutoTime();
		RobotMap.AutoBooleans.SHOOT_NOW = true;
		TrajectoryContainer.getInstance().trajectoryFollowing.resetAll();

		RobotMap.SensorMap.GYRO.zeroYaw();
		RobotMap.DrivetrainMap.ODOMETRY.resetPosition(new Pose2d(0.25, 2.29, new Rotation2d(0)), new Rotation2d(0));
		
	}

	@Override
	public void autonomousPeriodic() {
		RobotMap.StateChooser.FIELD_RELATIVE = false;

		System.out.println(RobotMap.DrivetrainMap.ODOMETRY.getPoseMeters());
		
		TrajectoryContainer.getInstance().trajectoryFollowing.auto();
		IntakeIndex.getInstance().autoIntake();
		//IntakeIndex.getInstance().index();
		Drivetrain.getInstance().updateOdometry();
		System.out.println("Total Time Seconds"
				+ TrajectoryContainer.getInstance().trajectoryFollowing.trajectory.getTotalTimeSeconds());
		System.out.println(
				"Total Time Seconds Robot" + TrajectoryContainer.getInstance().trajectoryFollowing.m_timer.get());
		
		
		//basicAuto.trajectoryPeriodic();
		//Shooter.getInstance().control();
		//IntakeIndex.getInstance().index();

		//MESSED WITH 2/1/2021
		


	}
	@Override
	public void teleopInit() {
		
		super.teleopInit();
		RobotMap.setDriveTalonsBrake();
		RobotMap.AutoBooleans.SHOOT_NOW = false;
		
	}
	@Override
	public void teleopPeriodic() {
		//RobotMap.StateChooser.FIELD_RELATIVE = false;

		Drivetrain.getInstance().teleopDrive();
		IntakeIndex.getInstance().twoBeltTwoBallIndex();
		Shooter.getInstance().control();
		Drivetrain.getInstance().updateOdometry();
		Climber.getInstance().climberPeriodic();

	}

	@Override
	public void testInit() {
		// Sets up the limelight pipeline
		// BallFollowDrive.initLimelight();


	}

	@Override
	public void testPeriodic() {
		/*
		 * // Button A on the XBOX makes the robot start searching again (if it marked
		 * // intake as done) if (RobotMap.CONTROLLER.getAButton()) {
		 * BallFollowDrive.resetIntakeState(); }
		 * 
		 * // Intake code runs only while the right bumber is held (otherwise it stops)
		 * if (RobotMap.CONTROLLER.getBumper(Hand.kRight)) { BallFollowDrive.drive(); }
		 * else { BallFollowDrive.stop(); }
		 */
		// Drivetrain.postToSmartDashboard();

		// RobotMap.StateChooser.RPM = 1100;
		//RobotMap.StateChooser.FIELD_RELATIVE = false;
		// Shooter.getInstance().control();
		Drivetrain.getInstance().teleopDrive();
		// IntakeIndex.getInstance().index();
		// Climber.getInstance().climberPeriodic();

		

		
	}
	
	@Override
	public void disabledInit() {
		
		super.disabledInit();
		//RobotMap.setDriveTalonsCoast();
		//TrajectoryContainer.getInstance().trajectoryFollowing.resetAll();
		
	}

}
