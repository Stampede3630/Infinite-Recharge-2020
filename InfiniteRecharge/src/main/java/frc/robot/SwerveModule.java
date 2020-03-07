/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveModule {

	public final WPI_TalonFX m_driveMotor;
	private final WPI_VictorSPX m_turningMotor;
	private double m_steeringAngle;
	private int m_driveScalar = 1;
	private final AnalogInput m_turningEncoder;
	private double stateDrive = 0;

	double driveOutput;

	// private final PIDController m_drivePIDController = new PIDController(0.25, 0,
	// 0);

	// private final ProfiledPIDController m_turningPIDController;// = new ProfiledPIDController(1, 0.0, 0.02, new
																// TrapezoidProfile.Constraints(kModuleMaxAngularVelocity,
	// kModuleMaxAngularAcceleration));//0.9 (.05 / Math.PI)

	// new TrapezoidProfile.Constraints(kModuleMaxAngularVelocity,
	// kModuleMaxAngularAcceleration)
	// front left -(0,1) - PID: 0.8, 0.05
	// (4,5) -PID: 0.8, 0.05
	private final PIDController m_turningPIDController;
	private final double angleOffset;

	/**
	 * Constructs a SwerveModule.
	 *
	 * @param driveMotorObject ID for the drive motor.
	 * @param steerMotorObject ID for the turning motor.
	 */

	public SwerveModule(WPI_TalonFX driveMotorObject, WPI_VictorSPX steerMotorObject,
			AnalogInput steerEncoderObject, double zeroedAngle) {
		double kPSpecial;
		double kD;
		// double kD;
		if (steerMotorObject.getDeviceID() == 4) {

			kPSpecial = .8;
			kD = 0.02;
			// kModuleMaxAngularVelocity = Math.PI;
			// kModuleMaxAngularAcceleration = Math.PI/2;
		} else {
			kPSpecial = 4/Math.PI;
			kD = 0.02;
			// kD = 0.02;
		}
			m_turningPIDController = new PIDController(kPSpecial, 0, kD);
		// m_turningPIDController = new ProfiledPIDController(kPSpecial, kI, 0.0,
		// 		new TrapezoidProfile.Constraints(RobotMap.SwerveModuleMap.MODULE_MAX_ANGULAR_VELOCITY,
		// 				RobotMap.SwerveModuleMap.MODULE_MAX_ANGULAR_ACCELERATION));

		m_driveMotor = driveMotorObject;
		m_turningMotor = steerMotorObject;
		m_turningEncoder = steerEncoderObject;

		angleOffset = zeroedAngle;

		// Set the distance per pulse for the drive encoder. We can simply use the
		// distance traveled for one rotation of the wheel divided by the encoder
		// loolution.

		// Set the distance (in this case, angle) per pulse for the turning encoder.
		// This is the the angle through an entire rotation (2 * wpi::math::pi)
		// divided by the encoder resolution.
		// m_turningEncoder.setDistancePerPulse(2 * Math.PI / kEncoderResolution);

		// Limit the PID Controller's input range between -pi and pi and set the input
		// to be continuous.
		m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

	}

	public double getTalonFXRate() {
		double ticksPerSec = m_driveMotor.getSelectedSensorVelocity(0) * 10;
		double revsPerSec = ticksPerSec / (RobotMap.SwerveModuleMap.ENCODER_RESOLUTION * 8.307692307692308);
		double metersPerSec = revsPerSec * 2 * Math.PI * RobotMap.SwerveModuleMap.WHEEL_RADIUS;
		return metersPerSec;
	}

	public double getTalonFXPos() {
		double ticks = m_driveMotor.getSelectedSensorPosition(0);
		double revs = ticks / (RobotMap.SwerveModuleMap.ENCODER_RESOLUTION * 8.307692307692308);
		double meters = revs * 2 * Math.PI * RobotMap.SwerveModuleMap.WHEEL_RADIUS;
		return meters;
	}

	/**
	 * Returns the current state of the module.
	 *
	 * @return The current state of the module.
	 */
	public SwerveModuleState getState() {
		return new SwerveModuleState(getTalonFXRate(), new Rotation2d(getAngle()));
	}

	public void readAngle() {
		double angle = ((1 - (m_turningEncoder.getVoltage() / RobotController.getVoltage5V())) * 2.0 * Math.PI
				+ angleOffset + 2 * Math.PI);
		angle %= 2 * Math.PI;
		angle -= Math.PI;
		m_steeringAngle = angle;
	}

	public double getAngle() {
		// readAngle();
		return m_steeringAngle;
	}

	public double angleSupp(double angle) {
		if (angle > Math.PI) {
			return angle - 2 * Math.PI;
		} else if (angle < -Math.PI) {
			return angle + 2 * Math.PI;
		} else {
			return angle;
		}
	}

	public double bound(double setpoint) {
		double dTheta = (setpoint + Math.PI) - (getAngle() + Math.PI); // saM THIS DOES ABSOLUTELY NOTHING. ADDING PI AND THEN REMOVING IT = 0.... - THANKS ANDY
		double trueDTheta = Math.IEEEremainder(dTheta, Math.PI);
		//double angleToReturn;
		
		/*if (Math.abs(Math.IEEEremainder(getAngle() + trueDTheta, 2 * Math.PI)
				- Math.IEEEremainder(setpoint, 2 * Math.PI)) < .01)*/
		if(Math.abs(Math.IEEEremainder(trueDTheta + getAngle() - setpoint, 2*Math.PI)) < .001) {
			m_driveScalar = 1;
		} else {
			m_driveScalar = -1;
		}

		//Ensure Angle is between -pi and pi
		if (Math.abs(trueDTheta) < Math.PI / 2) {
			return angleSupp(getAngle() + trueDTheta);
		} else {
			return angleSupp(getAngle() + (trueDTheta - Math.PI));
		}
	}
	public double getDriveOutput() {
		return driveOutput;
	}
	public double getWheelState()
	{
		return stateDrive;
	}

	/**
	 * Sets the desired state for the module.
	 *
	 * @param state Desired state with speed and angle.
	 */
	public void setDesiredState(SwerveModuleState state) {
		readAngle();
		m_driveScalar = 1;
		double setpoint = bound(state.angle.getRadians());
		double currentAngle = getAngle();
		var turnOutput = m_turningPIDController.calculate(currentAngle, setpoint);
		stateDrive = state.speedMetersPerSecond;
		driveOutput = state.speedMetersPerSecond / RobotMap.DriveMap.MAX_SPEED * m_driveScalar;
		// driveOutput = m_drivePIDController.calculate(Math.abs(getTalonFXRate()),
		// Math.abs(state.speedMetersPerSecond)) *
		// Math.signum(state.speedMetersPerSecond) *m_driveScalar;
    
		if (Math.abs(driveOutput)==0){
			turnOutput=0;
		}
	
		//SmartDashboard.putNumber("turnOutput", turnOutput);
		m_turningMotor.set(-turnOutput);
		m_driveMotor.set(-driveOutput);

		// System.out.println("measurement:" + currentAngle + ", setpoint: " +setpoint +
		// ", = (turn output) " + turnOutput);
		SmartDashboard.putNumber("Setpoint bound angle: " + m_driveMotor.getDeviceID(), Math.toDegrees(setpoint));
		SmartDashboard.putNumber("setpoint unbound angle: " + m_driveMotor.getDeviceID(), state.angle.getDegrees());
		SmartDashboard.putNumber("Current unbound angle: " + m_driveMotor.getDeviceID(), Math.toDegrees(getAngle()));
		SmartDashboard.putNumber("turn output: " + m_driveMotor.getDeviceID(), turnOutput);

	}

	public void toSmartDashboard() {
		SmartDashboard.putNumber("voltage" + m_driveMotor.getDeviceID(), m_turningEncoder.getVoltage());

	}
}