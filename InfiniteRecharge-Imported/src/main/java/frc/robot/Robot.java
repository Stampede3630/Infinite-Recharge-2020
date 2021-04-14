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
		TrajectoryContainer.getInstance().resetTimer();
		RobotMap.resetEncoders();
		// BallFollowDrive.resetIntakeState();

	}

	@Override
	public void robotPeriodic() {
		/*
		if(SmartDashboard.getBoolean("debugging", false))
		{
			Shooter.getInstance().smartDashboardOutput();
			Drivetrain.getInstance().postToSmartDashboard();
			
			ServoMotor.getInstance().setServoSmartDashboard();
			SmartDashboard.putNumber("Odometry X", RobotMap.DrivetrainMap.ODOMETRY.getPoseMeters().getTranslation().getX());
			SmartDashboard.putNumber("Odometry Y", RobotMap.DrivetrainMap.ODOMETRY.getPoseMeters().getTranslation().getY());
		}*/
		
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
		//SmartDashboard.putNumber("trajectory time", TrajectoryContainer.getInstance().trajectoryFollowing.trajectory.getTotalTimeSeconds());

		//emma was here 3/12/2021
		
		//ServoMotor.getInstance().setServoSmartDashboard();
	}

	@Override
	public void autonomousInit() {
		
		RobotMap.setDriveTalonsBrake();
		RobotMap.resetEncoders();
		basicAuto.resetAutoTime();
		RobotMap.AutoBooleans.SHOOT_NOW = true;
		RobotMap.StateConstants.ALLOW_AUTOMATED_CONTROL = true;

		
		TrajectoryContainer.getInstance().resetTimer();
		ChallengeAuto.getInstance().resetPathStep();
		

		RobotMap.SensorMap.GYRO.zeroYaw();

		//      						 GRAYSON PUT COORDS HERE  V     V
		RobotMap.DrivetrainMap.ODOMETRY.resetPosition(new Pose2d(1, 2.8, new Rotation2d(0)), new Rotation2d(0));
		
	}

	@Override
	public void autonomousPeriodic() {
		RobotMap.StateChooser.FIELD_RELATIVE = false;

		System.out.println(RobotMap.DrivetrainMap.ODOMETRY.getPoseMeters());
		
		//NAV CHALLENGE STUFF !!!!!!!
		//TrajectoryContainer.getInstance().trajectoryFollowingBarrelRoll.auto(); //1.19, 2.12
		//TrajectoryContainer.getInstance().trajectoryFollowingSlalom.auto(); //0.72, 0.75
		//TrajectoryContainer.getInstance().trajectoryFollowingBounce.auto(); //1.19, 2.29

		//GA STUFF !!!!!!
		//IntakeIndex.getInstance().autoIntake();
		//ChallengeAuto.getInstance().galacticSearchPeriodic();
		//TrajectoryContainer.getInstance().trajectoryFollowingGSBBlue.auto();
		// TrajectoryContainer.getInstance().trajectoryFollowingGSABlue.auto();
		// TrajectoryContainer.getInstance().trajectoryFollowingGSARed.auto();
		TrajectoryContainer.getInstance().trajectoryFollowingGSBRed.auto();


		
		Drivetrain.getInstance().updateOdometry();
		/*
		System.out.println("Total Time Seconds: "
				+ TrajectoryContainer.getInstance().trajectoryFollowing.trajectory.getTotalTimeSeconds());
		System.out.println(
				"Total Time Seconds Robot: " + TrajectoryContainer.getInstance().trajectoryFollowing.m_timer.get());
				*/
		
		
		//basicAuto.trajectoryPeriodic();
		//Shooter.getInstance().control();
		//IntakeIndex.getInstance().index();		


	}
	@Override
	public void teleopInit() {
		RobotMap.StateChooser.FIELD_RELATIVE = false;
		super.teleopInit();
		RobotMap.setDriveTalonsBrake();
		RobotMap.AutoBooleans.SHOOT_NOW = false;
		
	}
	@Override
	public void teleopPeriodic() {

		
		Drivetrain.getInstance().teleopDrive();
		IntakeIndex.getInstance().twoBeltTwoBallIndex();
		Shooter.getInstance().control();
		Drivetrain.getInstance().updateOdometry();
		//Climber.getInstance().climberPeriodic();

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
		//Drivetrain.getInstance().teleopDrive();
		// IntakeIndex.getInstance().index();
		// Climber.getInstance().climberPeriodic();

		//ServoMotor.getInstance().setServo(160);

		//Chooser.getInstance().autoChooser(Chooser.RobotState.GALACTIC_SEARCH);

		//RobotMap.IntakeMap.PINWHEEL.set(.6);
		RobotMap.IntakeMap.INTAKE_WHEELS.set(.8);


		
	}
	
	@Override
	public void disabledInit() {
		
		super.disabledInit();
		//RobotMap.setDriveTalonsCoast();
		//TrajectoryContainer.getInstance().trajectoryFollowing.resetAll();
		
	}

}
