/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.SupplyCurrentLimitConfiguration; //Emre
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.I2C;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.Victor;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

/**
 * Add your docs here.
 */
public class RobotMap {
	
	
	/**
	 * Talon SRX/ Victor SPX will supported multiple (cascaded) PID loops. For now
	 * we just want the primary one.
	 */
	public static final int kPIDLoopIdx = 0;
	/**
	 * Set to zero to skip waiting for confirmation, set to nonzero to wait and
	 * report to DS if action fails.
	 */
	public static final int kTimeoutMs = 30;

	/**
	 * TODO: What is this for?
	 */
	public static class AutoConstants { // Is this even being used?
		public static final double MAX_SPEED_METERS_PER_SECOND = 3;
		public static final double MAX_ACCELERATION_METERS_PER_SECOND_SQUARED = 3;
		public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND = Math.PI;
		public static final double MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED = Math.PI;

		public static final double KPX_CONTROLLER = 1.5;
		public static final double KPY_CONTROLLER = 1.5;
		public static final double KP_THETA_CONTROLLER = 1.5;



		// Constraint for the motion profilied robot angle controller
		public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
				MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED);
	}

	/**
	 * Constants used by BallFollowDrive
	 */
	public static class BallFollowMap {

		public static final int FLICKER_PROTECTION = 4;
		public static final double FAST_SEARCH = 0.9;
		public static final double SLOW_SEARCH = 0.6;
		public static final double FOLLOW_SPEED_MULTIPLIER = 0.7;

		// PID Constants/Contraints
		public static final double kMaxSpeed = 4; // 3 meters per second
		public static final double kMaxAngularSpeed = Math.PI;

		public static final double VELOCITY_MODIFIER_MULT = 0.5;
	}

	/**
	 * Constants for the Climber class
	 */
	public static class ClimberMap {
		public static final CANSparkMax TROLLEY_SPARK = new CANSparkMax(15, MotorType.kBrushless);
		public static final CANSparkMax ELEVATOR_SPARK = new CANSparkMax(14, MotorType.kBrushless);
		public static final DigitalInput MAX_LIMIT_SWITCH = new DigitalInput(1);
		public static final DigitalInput MIN_LIMIT_SWITCH = new DigitalInput(0);

		public static final double ELEVATOR_SPEED = -0.7;//0.7
		public static final double STRAFE_SPEED = 0.5;
	}

	/**
	 * Constants used by Drivetrain (swerve chassis)
	 */
	public static class DriveMap {
		public static final WPI_VictorSPX FRONT_RIGHT_ANGLE_MOTOR = new WPI_VictorSPX(2);
		public static final WPI_TalonFX FRONT_RIGHT_DRIVE_MOTOR = new WPI_TalonFX(1);
		public static final AnalogInput FRONT_RIGHT_ANGLE_ENCODER = new AnalogInput(0);
		public static final double FRONT_RIGHT_ANGLE_OFFSET = 2.521;

		public static final WPI_VictorSPX BACK_LEFT_ANGLE_MOTOR = new WPI_VictorSPX(6);
		public static final WPI_TalonFX BACK_LEFT_DRIVE_MOTOR = new WPI_TalonFX(5);
		public static final AnalogInput BACK_LEFT_ANGLE_ENCODER = new AnalogInput(1);
		public static final double BACK_LEFT_ANGLE_OFFSET = -4.0533;

		public static final WPI_VictorSPX BACK_RIGHT_ANGLE_MOTOR = new WPI_VictorSPX(4);
		public static final WPI_TalonFX BACK_RIGHT_DRIVE_MOTOR = new WPI_TalonFX(3);
		public static final AnalogInput BACK_RIGHT_ANGLE_ENCODER = new AnalogInput(2);
		public static final double BACK_RIGHT_ANGLE_OFFSET = -3.37466;

		public static final WPI_VictorSPX FRONT_LEFT_ANGLE_MOTOR = new WPI_VictorSPX(8);
		public static final WPI_TalonFX FRONT_LEFT_DRIVE_MOTOR = new WPI_TalonFX(7);
		public static final AnalogInput FRONT_LEFT_ANGLE_ENCODER = new AnalogInput(3);
		public static final double FRONT_LEFT_ANGLE_OFFSET = -2.2166453712; // radians

		public static final double MAX_SPEED = 3.627;
		public static final double MAX_ANGULAR_SPEED = 4 * Math.PI;

		public static final Translation2d FRONT_LEFT_LOCATION = new Translation2d(0.3556, 0.3556);
		public static final Translation2d FRONT_RIGHT_LOCATION = new Translation2d(0.3556, -0.3556);
		public static final Translation2d BACK_LEFT_LOCATION = new Translation2d(-0.3556, 0.3556);
		public static final Translation2d BACK_RIGHT_LOCATION = new Translation2d(-0.3556, -0.3556);
	
		public static final SwerveModule FRONT_LEFT = new SwerveModule(FRONT_LEFT_DRIVE_MOTOR,
				FRONT_LEFT_ANGLE_MOTOR, FRONT_LEFT_ANGLE_ENCODER,
				FRONT_LEFT_ANGLE_OFFSET);
		public static final SwerveModule FRONT_RIGHT = new SwerveModule(FRONT_RIGHT_DRIVE_MOTOR,
				FRONT_RIGHT_ANGLE_MOTOR,FRONT_RIGHT_ANGLE_ENCODER,
				FRONT_RIGHT_ANGLE_OFFSET);
		public static final SwerveModule BACK_LEFT = new SwerveModule(BACK_LEFT_DRIVE_MOTOR,
				BACK_LEFT_ANGLE_MOTOR, BACK_LEFT_ANGLE_ENCODER,
				BACK_LEFT_ANGLE_OFFSET);
		public static final SwerveModule BACK_RIGHT = new SwerveModule(BACK_RIGHT_DRIVE_MOTOR,
				BACK_RIGHT_ANGLE_MOTOR, BACK_RIGHT_ANGLE_ENCODER,
				BACK_RIGHT_ANGLE_OFFSET);
	
		public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(FRONT_LEFT_LOCATION,
				FRONT_RIGHT_LOCATION, BACK_LEFT_LOCATION, BACK_RIGHT_LOCATION);
	
		public static final SwerveDriveOdometry ODOMETRY = new SwerveDriveOdometry(KINEMATICS,
				Rotation2d.fromDegrees(-SensorMap.GYRO.getAngle()));
	
	}

	/**
	 * Limelight pipeline constants
	 */
	public static class Pipelines {
		public static final int BALL_FOLLOW = 1;
		public static final int TARGET_LINEUP = 3;
		public static final int TARGET_LINEUP_ZOOM = 4;
	}

	/**
	 * Constants used by TargetAlignDrive
	 */
	public static class TargetAlignMap {
		public static final double ANGLE_THRESHOLD = 1;
	}


	/**
	 * Constants for sensors
	 */
	public static class SensorMap {
		public static final AHRS GYRO = new AHRS(SPI.Port.kMXP);

	}

	/**
	 * Constants used by the Shooter class
	 */


	public static class SwerveModuleMap {
		public static final double WHEEL_RADIUS = 0.051;
		public static final int ENCODER_RESOLUTION = 2048;
		public static double MODULE_MAX_ANGULAR_VELOCITY = Math.PI * 2 * 2.6;
		public static double MODULE_MAX_ANGULAR_ACCELERATION = 6 * Math.PI; // radians per second squared //4
	
	}

	public static class StateChooser
	{
		public static double LIMELIGHT_ANGLE;
		public static double kF = 0.055;
		public static double kP = 1;
		public static double RPM;
		public static boolean HOOD_ANGLE;
		public static int PIPELINE;
		public static double DRIVE_ANGLE;
		public static boolean FIELD_RELATIVE = true; 
	}


	public static class AutoBooleans
	{
		public static boolean INTAKE_NOW = false;
		public static boolean SHOOT_NOW = false;
		public static boolean TRAJECTORY_DONE = false;
	}

	public static final XboxController CONTROLLER = new XboxController(0);

	static {
		// DriveMap
		DriveMap.FRONT_LEFT_DRIVE_MOTOR.setInverted(true);//f
		DriveMap.BACK_LEFT_DRIVE_MOTOR.setInverted(true);//f
		DriveMap.FRONT_RIGHT_DRIVE_MOTOR.setInverted(false);//t
		DriveMap.BACK_RIGHT_DRIVE_MOTOR.setInverted(false);//t



		//Current limiting for motors
		DriveMap.FRONT_LEFT_DRIVE_MOTOR.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 0, 0.1));
		DriveMap.FRONT_RIGHT_DRIVE_MOTOR.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 0, 0.1));
		DriveMap.BACK_LEFT_DRIVE_MOTOR.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 0, 0.1));
		DriveMap.BACK_RIGHT_DRIVE_MOTOR.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 30, 0, 0.1));

		DriveMap.BACK_LEFT_ANGLE_MOTOR.setInverted(true);
		DriveMap.FRONT_LEFT_ANGLE_MOTOR.setInverted(true);
		DriveMap.BACK_RIGHT_ANGLE_MOTOR.setInverted(true);
		DriveMap.FRONT_RIGHT_ANGLE_MOTOR.setInverted(true);


		DriveMap.FRONT_LEFT_DRIVE_MOTOR.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, kPIDLoopIdx,
		kTimeoutMs); // setting to integrated sensor
		DriveMap.BACK_LEFT_DRIVE_MOTOR.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, kPIDLoopIdx,
		kTimeoutMs);
		DriveMap.FRONT_RIGHT_DRIVE_MOTOR.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, kPIDLoopIdx,
		kTimeoutMs);
		DriveMap.BACK_RIGHT_DRIVE_MOTOR.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, kPIDLoopIdx,
		kTimeoutMs);

		// Shooter

		
	}

	public static void resetEncoders() {
		DriveMap.BACK_RIGHT_DRIVE_MOTOR.setSelectedSensorPosition(0);
		DriveMap.FRONT_RIGHT_DRIVE_MOTOR.setSelectedSensorPosition(0);
		DriveMap.BACK_LEFT_DRIVE_MOTOR.setSelectedSensorPosition(0);
		DriveMap.FRONT_LEFT_DRIVE_MOTOR.setSelectedSensorPosition(0);
	}

	public static void setDriveTalonsBrake()
	{
		DriveMap.FRONT_LEFT_DRIVE_MOTOR.setNeutralMode(NeutralMode.Brake);
		DriveMap.BACK_LEFT_DRIVE_MOTOR.setNeutralMode(NeutralMode.Brake);
		DriveMap.FRONT_RIGHT_DRIVE_MOTOR.setNeutralMode(NeutralMode.Brake);
		DriveMap.BACK_RIGHT_DRIVE_MOTOR.setNeutralMode(NeutralMode.Brake);

		DriveMap.BACK_LEFT_ANGLE_MOTOR.setNeutralMode(NeutralMode.Brake);
		DriveMap.FRONT_LEFT_ANGLE_MOTOR.setNeutralMode(NeutralMode.Brake);
		DriveMap.BACK_RIGHT_ANGLE_MOTOR.setNeutralMode(NeutralMode.Brake);
		DriveMap.FRONT_RIGHT_ANGLE_MOTOR.setNeutralMode(NeutralMode.Brake);
	}

	public static void setDriveTalonsCoast()
	{
		DriveMap.FRONT_LEFT_DRIVE_MOTOR.setNeutralMode(NeutralMode.Coast);
		DriveMap.BACK_LEFT_DRIVE_MOTOR.setNeutralMode(NeutralMode.Coast);
		DriveMap.FRONT_RIGHT_DRIVE_MOTOR.setNeutralMode(NeutralMode.Coast);
		DriveMap.BACK_RIGHT_DRIVE_MOTOR.setNeutralMode(NeutralMode.Coast);

		DriveMap.BACK_LEFT_ANGLE_MOTOR.setNeutralMode(NeutralMode.Coast);
		DriveMap.FRONT_LEFT_ANGLE_MOTOR.setNeutralMode(NeutralMode.Coast);
		DriveMap.BACK_RIGHT_ANGLE_MOTOR.setNeutralMode(NeutralMode.Coast);
		DriveMap.FRONT_RIGHT_ANGLE_MOTOR.setNeutralMode(NeutralMode.Coast);
	}

	
	
	
	

	
}
