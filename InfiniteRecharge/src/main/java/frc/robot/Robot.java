/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrajectoryParameterizer;
import frc.robot.RobotMap.DriveMap;


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
		SmartDashboard.putBoolean("debugging", false);
		// BallFollowDrive.resetIntakeState();
	}

	@Override
	public void robotPeriodic() {
		if(SmartDashboard.getBoolean("debugging", false))
		{
			Drivetrain.getInstance().postToSmartDashboard();
			SmartDashboard.putNumber("Odometry X", -RobotMap.DriveMap.ODOMETRY.getPoseMeters().getTranslation().getX());
			SmartDashboard.putNumber("Odometry Y", RobotMap.DriveMap.ODOMETRY.getPoseMeters().getTranslation().getY());
		}
		

		Chooser.getInstance().chooserPeriodic();
	}

	@Override
	public void autonomousInit() {

		RobotMap.setDriveTalonsBrake();
		RobotMap.resetEncoders();
		TrajectoryContainer.getInstance().trajectoryFollowing.resetAll();
		basicAuto.resetAutoTime();
		RobotMap.AutoBooleans.SHOOT_NOW = true;
	}

	@Override
	public void autonomousPeriodic() {
		/*
		TrajectoryContainer.getInstance().trajectoryFollowing.auto();
		IntakeIndex.getInstance().index();
		Drivetrain.getInstance().updateOdometry();
		System.out.println("Total Time Seconds"
				+ TrajectoryContainer.getInstance().trajectoryFollowing.trajectory.getTotalTimeSeconds());
		System.out.println(
				"Total Time Seconds Robot" + TrajectoryContainer.getInstance().trajectoryFollowing.m_timer.get());
		*/
		basicAuto.trajectoryPeriodic();
		Drivetrain.getInstance().updateOdometry();

	}
	@Override
	public void teleopInit() {
		super.teleopInit();
		RobotMap.setDriveTalonsBrake();
		RobotMap.AutoBooleans.SHOOT_NOW = false;
	}
	@Override
	public void teleopPeriodic() {

		Drivetrain.getInstance().teleopDrive();
		

	}

	@Override
	public void testInit() {
		// Sets up the limelight pipeline
		// BallFollowDrive.initLimelight();


	}

	@Override
	public void testPeriodic() {

	}

	@Override
	public void disabledInit() {

	}

}
