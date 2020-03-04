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
import com.revrobotics.ColorSensorV3;
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
		public static final double FRONT_RIGHT_ANGLE_OFFSET = 2.572186-0.05111283669824829;

		public static final WPI_VictorSPX BACK_LEFT_ANGLE_MOTOR = new WPI_VictorSPX(6);
		public static final WPI_TalonFX BACK_LEFT_DRIVE_MOTOR = new WPI_TalonFX(5);
		public static final AnalogInput BACK_LEFT_ANGLE_ENCODER = new AnalogInput(1);
		public static final double BACK_LEFT_ANGLE_OFFSET = -0.925161-3.128168277531419;

		public static final WPI_VictorSPX BACK_RIGHT_ANGLE_MOTOR = new WPI_VictorSPX(4);
		public static final WPI_TalonFX BACK_RIGHT_DRIVE_MOTOR = new WPI_TalonFX(3);
		public static final AnalogInput BACK_RIGHT_ANGLE_ENCODER = new AnalogInput(2);
		public static final double BACK_RIGHT_ANGLE_OFFSET = -0.420229 - 0.8095408897871463-2.1448951380578576;

		public static final WPI_VictorSPX FRONT_LEFT_ANGLE_MOTOR = new WPI_VictorSPX(8);
		public static final WPI_TalonFX FRONT_LEFT_DRIVE_MOTOR = new WPI_TalonFX(7);
		public static final AnalogInput FRONT_LEFT_ANGLE_ENCODER = new AnalogInput(3);
		public static final double FRONT_LEFT_ANGLE_OFFSET = -2.21-0.006645810835413712; // radians

		public static final double MAX_SPEED = 3.627;
		public static final double MAX_ANGULAR_SPEED = 4 * Math.PI;
	}

	public static class DrivetrainMap {
		public static final Translation2d FRONT_LEFT_LOCATION = new Translation2d(0.3556, 0.3556);
		public static final Translation2d FRONT_RIGHT_LOCATION = new Translation2d(0.3556, -0.3556);
		public static final Translation2d BACK_LEFT_LOCATION = new Translation2d(-0.3556, 0.3556);
		public static final Translation2d BACK_RIGHT_LOCATION = new Translation2d(-0.3556, -0.3556);
	
		public static final SwerveModule FRONT_LEFT = new SwerveModule(RobotMap.DriveMap.FRONT_LEFT_DRIVE_MOTOR,
				RobotMap.DriveMap.FRONT_LEFT_ANGLE_MOTOR, RobotMap.DriveMap.FRONT_LEFT_ANGLE_ENCODER,
				RobotMap.DriveMap.FRONT_LEFT_ANGLE_OFFSET);
		public static final SwerveModule FRONT_RIGHT = new SwerveModule(RobotMap.DriveMap.FRONT_RIGHT_DRIVE_MOTOR,
				RobotMap.DriveMap.FRONT_RIGHT_ANGLE_MOTOR,RobotMap.DriveMap.FRONT_RIGHT_ANGLE_ENCODER,
				RobotMap.DriveMap.FRONT_RIGHT_ANGLE_OFFSET);
		public static final SwerveModule BACK_LEFT = new SwerveModule(RobotMap.DriveMap.BACK_LEFT_DRIVE_MOTOR,
				RobotMap.DriveMap.BACK_LEFT_ANGLE_MOTOR, RobotMap.DriveMap.BACK_LEFT_ANGLE_ENCODER,
				RobotMap.DriveMap.BACK_LEFT_ANGLE_OFFSET);
		public static final SwerveModule BACK_RIGHT = new SwerveModule(RobotMap.DriveMap.BACK_RIGHT_DRIVE_MOTOR,
				RobotMap.DriveMap.BACK_RIGHT_ANGLE_MOTOR, RobotMap.DriveMap.BACK_RIGHT_ANGLE_ENCODER,
				RobotMap.DriveMap.BACK_RIGHT_ANGLE_OFFSET);
	
		public static final SwerveDriveKinematics KINEMATICS = new SwerveDriveKinematics(FRONT_LEFT_LOCATION,
				FRONT_RIGHT_LOCATION, BACK_LEFT_LOCATION, BACK_RIGHT_LOCATION);
	
		public static final SwerveDriveOdometry ODOMETRY = new SwerveDriveOdometry(KINEMATICS,
				Rotation2d.fromDegrees(-SensorMap.GYRO.getAngle()));

	}

	public static class IntakeMap {

		public static final CANSparkMax INTAKE_WHEELS = new CANSparkMax(9, MotorType.kBrushless);// first spin wheel
		public static final WPI_TalonSRX PINWHEEL = new WPI_TalonSRX(10); // from box to belt
		public static final DoubleSolenoid ARMS_SOLENOID = new DoubleSolenoid(0, 1); // lowers the arms
		public static final DoubleSolenoid HOOD_ANGLE = new DoubleSolenoid(6, 7);
		//public static final Ultrasonic ULTRASONIC = new Ultrasonic(9, 8); // on the ground of the belt box
		public static final WPI_TalonFX BELT = new WPI_TalonFX(11);
		// public static final ColorSensorV3 COLOR_SENSOR_MID = new
		// ColorSensorV3(I2C.Port.kOnboard);
		// public static final ColorSensorV3 COLOR_SENSOR_HIGH = new
		// ColorSensorV3(I2C.Port.kMXP);
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
	 * PIDs used by target tracking classes
	 */
	public static class TrackingPIDMap {
		public static final ProfiledPIDController TURN = new ProfiledPIDController(0.01, 0, 0,
				new TrapezoidProfile.Constraints(RobotMap.DriveMap.MAX_ANGULAR_SPEED, Math.PI * 6)); // TODO?
		public static final PIDController X = new PIDController(0.03, 0, 0);
		public static final PIDController Y = new PIDController(0.03, 0, 0);
		public static final PIDController X_VEL = new PIDController(0.03, 0, 0);
		public static final PIDController Y_VEL = new PIDController(0.03, 0, 0);
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
	public static class ShooterMap {
		public static final WPI_TalonFX LEFT_SHOOTER_FALCON = new WPI_TalonFX(13);
		public static final WPI_TalonFX RIGHT_SHOOTER_FALCON = new WPI_TalonFX(12);
		
		public static final double RPM = SmartDashboard.getNumber("RPM", 3600); //Renamed from rotpm
		/**
		 * Which PID slot to pull gains from. Starting 2018, you can choose from 0,1,2
		 * or 3. Only the first two (0,1) are visible in web-based configuration.
		 */
		public static final int SLOT_IDX = 0; // TODO: UNUSED

		/**
		 * Talon SRX/ Victor SPX will supported multiple (cascaded) PID loops. For now
		 * we just want the primary one.
		 */
		public static final int PID_LOOP_IDX = 0;
		/**
		 * Set to zero to skip waiting for confirmation, set to nonzero to wait and
		 * report to DS if action fails.
		 */
		public static final int TIMEOUT_MS = 50;

		public static final double ENCODER_UNITS_PER_REV = 2048;// 4096;
	}

	public static class SwerveModuleMap {
		public static final double WHEEL_RADIUS = 0.051;
		public static final int ENCODER_RESOLUTION = 2048;
		public static double MODULE_MAX_ANGULAR_VELOCITY = Math.PI * 2 * 2.6;
		public static double MODULE_MAX_ANGULAR_ACCELERATION = 6 * Math.PI; // radians per second squared //4
	
	}

	public static class StateChooser
	{
		public static double LIMELIGHT_ANGLE;
		public static double kF;
		public static double RPM;
		public static boolean HOOD_ANGLE;
		public static int PIPELINE;
		public static double DRIVE_ANGLE;
		public static boolean FIELD_RELATIVE = false;
	}

	public static class StateConstants
	{
		public static final double CLIMBER_SERVO_POS = 0 ;
		public static final double LEFT_CLIMBER_ANGLE = 22.5 * (Math.PI/180);
		public static final double RIGHT_CLIMBER_ANGLE = (22.5 - 180) * (Math.PI/180);
		public static final int CLIMBER_PIPELINE = 1;

		public static double INTAKE_SERVO_POS;
        public static double INTAKE_ANGLE = Math.PI/2;
        public static int INTAKE_PIPELINE = 1;

		public static double INITIATION_LINE_SHOT_SERVO_ANGLE = 45;//real angle:30
        public static double INITIATION_LINE_SHOT_ANGLE = 999;//trig
        public static int INITIATION_LINE_SHOT_PIPELINE = 3;						//DONE
        public static double INITIATION_LINE_SHOT_KF = 0.055;
		public static int INITIATION_LINE_SHOT_RPM = 3050; //top of range
		public static boolean INITIATION_LINE_SHOT_HOOD_ANGLE = false; //false is high angle
		
		public static double SHORT_TRENCH_AUTO_SERVO_ANGLE = 40;//real angle:20
        public static double SHORT_TRENCH_AUTO_ANGLE = 24.61 * (Math.PI/180);//TRIG
        public static int SHORT_TRENCH_AUTOT_PIPELINE = 3;
        public static double SHORT_TRENCH_AUTOT_KF = 0.055;							//DONE except trig
		public static int SHORT_TRENCH_AUTO_RPM = 3500; //top of range
        public static boolean SHORT_TRENCH_AUTO_HOOD_ANGLE = true; //false is high angle

        public static double SHORT_TRENCH_SERVO_ANGLE = 40;
        public static double SHORT_TRENCH_ANGLE = 24.61 * (Math.PI/180); //ish
		public static int SHORT_TRENCH_PIPELINE = 3;
        public static double SHORT_TRENCH_KF = 0.055;
		public static int SHORT_TRENCH_RPM = 3800; //3825 for bad balls
        public static boolean SHORT_TRENCH_HOOD_ANGLE = true;

        public static double LONG_SHOT_SERVO_POS;
		public static double LONG_SHOT_ANGLE = 11 * (Math.PI/180);
		public static int LONG_SHOT_PIPELINE = 4;
        public static double LONG_SHOT_KF = 0.06;
		public static int LONG_SHOT_RPM = 4000;
		public static boolean LONG_SHOT_HOOD_ANGLE = true;
		
		public static double NO_MANS_LAND_SERVO_POS;
		public static double NO_MANS_LAND_ANGLE = 999;
		public static int NO_MANS_LAND_PIPELINE = 3;
        public static double NO_MANS_LAND_KF = 0.055;
		public static int NO_MANS_LAND_RPM = 3800;
		public static boolean NO_MANS_LAND_HOOD_ANGLE = true;
		
		public static boolean ALLOW_AUTOMATED_CONTROL = true;

	}

	public static class AutoBooleans
	{
		public static boolean INTAKE_NOW = false;
		public static boolean SHOOT_NOW = false;
	}

	public static final XboxController CONTROLLER = new XboxController(0);

	static {
		// DriveMap
		DriveMap.FRONT_LEFT_DRIVE_MOTOR.setInverted(true);
		DriveMap.BACK_LEFT_DRIVE_MOTOR.setInverted(true);
		DriveMap.FRONT_RIGHT_DRIVE_MOTOR.setInverted(false);
		DriveMap.BACK_RIGHT_DRIVE_MOTOR.setInverted(false);



		//Current limiting for motors
		DriveMap.FRONT_LEFT_DRIVE_MOTOR.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 45, 0.5));
		DriveMap.FRONT_RIGHT_DRIVE_MOTOR.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 45, 0.5));
		DriveMap.BACK_LEFT_DRIVE_MOTOR.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 45, 0.5));
		DriveMap.BACK_RIGHT_DRIVE_MOTOR.configSupplyCurrentLimit(new SupplyCurrentLimitConfiguration(true, 40, 45, 0.5));

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

		ShooterMap.LEFT_SHOOTER_FALCON.configFactoryDefault();
		ShooterMap.RIGHT_SHOOTER_FALCON.configFactoryDefault();

		IntakeMap.BELT.setNeutralMode(NeutralMode.Brake);

		ShooterMap.LEFT_SHOOTER_FALCON.setInverted(false);
		ShooterMap.RIGHT_SHOOTER_FALCON.set(ControlMode.Follower, ShooterMap.LEFT_SHOOTER_FALCON.getDeviceID());
		ShooterMap.RIGHT_SHOOTER_FALCON.setInverted(InvertType.OpposeMaster);
		
		ShooterMap.LEFT_SHOOTER_FALCON.enableVoltageCompensation(true);
		ShooterMap.LEFT_SHOOTER_FALCON.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, kPIDLoopIdx,
				kTimeoutMs);
		ShooterMap.LEFT_SHOOTER_FALCON.setSensorPhase(true);
		//ShooterMap.LEFT_SHOOTER_FALCON.setSelectedSensorPosition(1);  THE INTENT WAS TO SET TO INTEGRATED SENSOR

		/* Config the peak and nominal outputs */
		ShooterMap.LEFT_SHOOTER_FALCON.configNominalOutputForward(0, kTimeoutMs);
		ShooterMap.LEFT_SHOOTER_FALCON.configNominalOutputReverse(0, kTimeoutMs);
		ShooterMap.LEFT_SHOOTER_FALCON.configPeakOutputForward(1, kTimeoutMs);
		ShooterMap.LEFT_SHOOTER_FALCON.configPeakOutputReverse(-1, kTimeoutMs);

		/* Config the Velocity closed loop gains in slot0 */
		ShooterMap.LEFT_SHOOTER_FALCON.config_kF(kPIDLoopIdx, 0.055, kTimeoutMs); // .45 *(1023.0/7200.0)
		ShooterMap.LEFT_SHOOTER_FALCON.config_kP(kPIDLoopIdx, 0.4, kTimeoutMs);
		ShooterMap.LEFT_SHOOTER_FALCON.config_kI(kPIDLoopIdx, 0, kTimeoutMs);
		ShooterMap.LEFT_SHOOTER_FALCON.config_kD(kPIDLoopIdx, 0, kTimeoutMs);

		// Intake

		//IntakeMap.ULTRASONIC.setAutomaticMode(true);
		//IntakeMap.ULTRASONIC.setDistanceUnits(Ultrasonic.Unit.kInches);
		// IntakeMap.COLOR_SENSOR_MID.setAutomaticMode(true);
		// IntakeMap.COLOR_SENSOR_MID.setDistanceUnits(Ultrasonic.Unit.kInches);
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
