/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
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
	/**
	 * This function is run when the robot is first started up and should be used
	 * for any initialization code.
	 */
	double priorAutospeed = 0;
	Number[] numberArray = new Number[10];
	NetworkTableEntry autoSpeedEntry = NetworkTableInstance.getDefault().getEntry("/robot/autospeed");
	NetworkTableEntry telemetryEntry = NetworkTableInstance.getDefault().getEntry("/robot/telemetry");
	NetworkTableEntry rotateEntry = NetworkTableInstance.getDefault().getEntry("/robot/rotate");

	public static final double kMaxSpeed = 4; // 3 meters per second
	public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second
	public static XboxController m_controller;
	TrajectoryFollowing autonomous;
	TestManipulator manipTest;
	// Shooter shooter;

	@Override
	public void robotInit() {
		m_controller = new XboxController(0);
		manipTest = new TestManipulator();
		// shooter = new Shooter();
		BallFollowDrive.resetIntakeState();
	}

	@Override
	public void robotPeriodic() {
		Drivetrain.postToSmartDashboard();
		Drivetrain.updateOdometry();
		// shooter.smartDashboardOutput();

	}

	@Override
	public void autonomousInit() {
		NetworkTableInstance.getDefault().setUpdateRate(0.010);
		if (!isReal())
			SmartDashboard.putData(new SimEnabler());
		BallFollowDrive.initLimelight();
		BallFollowDrive.resetIntakeState();
	}

	@Override
	public void autonomousPeriodic() {

		// BallFollowDrive.intake(kMaxAngularSpeed / 2);

		// // Retrieve values to send back before telling the motors to do something
		// double now = Timer.getFPGATimestamp();

		// double leftPosition = -m_swerve.m_frontLeft.getTalonFXPos();
		// double leftRate = -m_swerve.m_frontLeft.getTalonFXRate();
		// //System.out.println(m_swerve.m_frontLeft.m_driveMotor.getSelectedSensorPosition());

		// double rightPosition = -m_swerve.m_frontRight.getTalonFXPos();
		// double rightRate = -m_swerve.m_frontRight.getTalonFXRate();

		// double battery = RobotController.getBatteryVoltage();

		// double leftMotorVolts =
		// 0;//m_swerve.m_frontLeft.m_driveMotor.getMotorOutputVoltage();
		// double rightMotorVolts =
		// 0;//m_swerve.m_frontRight.m_driveMotor.getMotorOutputVoltage();

		// // Retrieve the commanded speed from NetworkTables
		// double autospeed = autoSpeedEntry.getDouble(0);
		// priorAutospeed = autospeed;

		// // command motors to do things
		// double xspeed, yspeed, rot;
		// boolean fieldRelative;
		// if(rotateEntry.getBoolean(false)){

		// xspeed = 0;
		// yspeed = 0;
		// rot = autospeed;
		// fieldRelative = false;
		// } else {
		// xspeed = autospeed;
		// yspeed = 0;
		// rot = 0;
		// fieldRelative = false;
		// }
		// m_swerve.drive(xspeed, yspeed, rot, fieldRelative);

		// // send telemetry data array back to NT
		// numberArray[0] = now;
		// numberArray[1] = battery;
		// numberArray[2] = autospeed;
		// numberArray[3] = leftMotorVolts;
		// numberArray[4] = rightMotorVolts;
		// numberArray[5] = leftPosition;
		// numberArray[6] = rightPosition;
		// numberArray[7] = leftRate;
		// numberArray[8] = rightRate;
		// numberArray[9] = m_swerve.getAngle().getRadians();

		// telemetryEntry.setNumberArray(numberArray);

	}

	@Override
	public void teleopInit() {
	}

	@Override
	public void teleopPeriodic() {

		driveWithJoystick(false);
		/*
		 * manipTest.periodic(); if(m_controller.getTriggerAxis(Hand.kLeft)>0.5) {
		 * shooter.control(); } else { shooter.drive(); }
		 * 
		 */
	}

	@Override
	public void testInit() {
	}

	@Override
	public void testPeriodic() {

		if (m_controller.getAButton()) {
			BallFollowDrive.resetIntakeState();
		}
		if (m_controller.getBumper(Hand.kRight)) {
			BallFollowDrive.intake(kMaxAngularSpeed / 2);
		} else {
			BallFollowDrive.stop();
		}

		Drivetrain.postToSmartDashboard();

	}

	private void driveWithJoystick(boolean fieldRelative) {
		// Get the x speed. We are inverting this because Xbox controllers return
		// negative values when we push forward.
		var xSpeed = -MathHelper.deadzone(m_controller.getY(Hand.kLeft), 0.2) * kMaxSpeed;

		// Get the y speed or sideways/strafe speed. We are inverting this because
		// we want a positive value when we pull to the left. Xbox controllers
		// return positive values when you pull to the right by default.
		var ySpeed = -MathHelper.deadzone(m_controller.getX(Hand.kLeft), 0.2) * kMaxSpeed;

		// Get the rate of angular rotation. We are inverting this because we want a
		// positive value when we pull to the left (remember, CCW is positive in
		// mathematics). Xbox controllers return positive values when you pull to
		// the right by default.
		var rot = -MathHelper.deadzone(m_controller.getX(Hand.kRight), 0.2) * kMaxAngularSpeed;

		// System.out.println("rot: " + m_controller.getX(Hand.kRight));
		// System.out.println("rot-c: " + rot);
		System.out.println(xSpeed + "," + ySpeed);
		Drivetrain.drive(xSpeed, ySpeed, rot, fieldRelative);
	}
}