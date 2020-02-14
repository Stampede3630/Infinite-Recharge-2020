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
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

/**
 * Add your docs here.
 */
public class RobotMap {
    // shooter stuff
    // public static WPI_TalonFX leftShooterFalcon = new WPI_TalonFX(12);
    // public static WPI_TalonFX rightShooterFalcon = new WPI_TalonFX(13);
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

        public static final double KPX_CONTROLLER = 1;
        public static final double KPY_CONTROLLER = 1;
        public static final double KP_THETA_CONTROLLER = 1;

        // Constraint for the motion profilied robot angle controller
        public static final TrapezoidProfile.Constraints kThetaControllerConstraints = new TrapezoidProfile.Constraints(
                MAX_ANGULAR_SPEED_RADIANS_PER_SECOND, MAX_ANGULAR_SPEED_RADIANS_PER_SECOND_SQUARED);
    }

    /**
     * Constants for the Climber class
     */
    public static class ClimberMap {
        public static final CANSparkMax TROLLEY_SPARK = new CANSparkMax(15, MotorType.kBrushless);
        public static final CANSparkMax ELEVATOR_SPARK = new CANSparkMax(14, MotorType.kBrushless);
        public static final DigitalInput MAX_LIMIT_SWITCH = new DigitalInput(1);
        public static final DigitalInput MIN_LIMIT_SWITCH = new DigitalInput(0);
    }

    /**
     * PIDs used by target tracking classes
     */
    public static class TrackingPIDMap {
        public static final ProfiledPIDController TURN = new ProfiledPIDController(0.03, 0, 0,
                new TrapezoidProfile.Constraints(RobotMap.PIDConstraints.MAX_ANGULAR_SPEED, Math.PI * 6)); // TODO?
        public static final PIDController X = new PIDController(0.03, 0, 0);
        public static final PIDController Y = new PIDController(0.03, 0, 0);
        public static final PIDController X_VEL = new PIDController(0.03, 0, 0);
        public static final PIDController Y_VEL = new PIDController(0.03, 0, 0);
    }

    /**
     * Constants used by the Shooter class
     */
    public static class ShooterMap {
        public static final WPI_TalonFX LEFT_SHOOTER_FALCON = new WPI_TalonFX(12);
        public static final WPI_TalonFX RIGHT_SHOOTER_FALCON = new WPI_TalonFX(13);
        public static final WPI_TalonSRX BELT = new WPI_TalonSRX(11);
    }

    public static class IntakeMap {

        public static final CANSparkMax INTAKE_WHEELS = new CANSparkMax(9, MotorType.kBrushless);// first spin wheel
        public static final WPI_TalonSRX PINWHEEL = new WPI_TalonSRX(10); // from box to belt
        public static final DoubleSolenoid ARMS_SOLENOID = new DoubleSolenoid(0, 1); // lowers the arms
        public static final DoubleSolenoid HOOD_ANGLE = new DoubleSolenoid(6,7);
        public static final Ultrasonic ULTRASONIC = new Ultrasonic(7, 8); // on the ground of the belt box
        public static final ColorSensorV3 COLOR_SENSOR_MID = new ColorSensorV3(I2C.Port.kOnboard);
        public static final ColorSensorV3 COLOR_SENSOR_HIGH = new ColorSensorV3(I2C.Port.kMXP);
    }

    /**
     * Constants for sensors
     */
    public static class SensorMap {
        public static final AHRS GYRO = new AHRS(SPI.Port.kMXP);
    }

    /**
     * Constants used by Drivetrain (swerve chassis)
     */
    public static class DriveMap {
        public static final WPI_VictorSPX FRONT_RIGHT_ANGLE_MOTOR = new WPI_VictorSPX(2);
        public static final WPI_TalonFX FRONT_RIGHT_DRIVE_MOTOR = new WPI_TalonFX(1);
        public static final AnalogInput FRONT_RIGHT_ANGLE_ENCODER = new AnalogInput(0);
        public static final double FRONT_RIGHT_ANGLE_OFFSET = 0;

        public static final WPI_VictorSPX BACK_LEFT_ANGLE_MOTOR = new WPI_VictorSPX(6);
        public static final WPI_TalonFX BACK_LEFT_DRIVE_MOTOR = new WPI_TalonFX(5);
        public static final AnalogInput BACK_LEFT_ANGLE_ENCODER = new AnalogInput(1);
        public static final double BACK_LEFT_ANGLE_OFFSET = 0;

        public static final WPI_VictorSPX BACK_RIGHT_ANGLE_MOTOR = new WPI_VictorSPX(4);
        public static final WPI_TalonFX BACK_RIGHT_DRIVE_MOTOR = new WPI_TalonFX(3);
        public static final AnalogInput BACK_RIGHT_ANGLE_ENCODER = new AnalogInput(2);
        public static final double BACK_RIGHT_ANGLE_OFFSET = 0;

        public static final WPI_VictorSPX FRONT_LEFT_ANGLE_MOTOR = new WPI_VictorSPX(8);
        public static final WPI_TalonFX FRONT_LEFT_DRIVE_MOTOR = new WPI_TalonFX(7);
        public static final AnalogInput FRONT_LEFT_ANGLE_ENCODER = new AnalogInput(3);
        public static final double FRONT_LEFT_ANGLE_OFFSET = 0; // radians
    }

    /**
     * TODO: What is this for?\
     */
    public static class PIDConstraints {
        // PID Constants/Contraints
        public static final double MAX_SPEED = 4; // 3 meters per second
        public static final double MAX_ANGULAR_SPEED = Math.PI;
    }

    /**
     * Limelight pipeline constants
     */
    public static class Pipelines {
        public static final int BALL_FOLLOW = 1;
        public static final int TARGET_LINEUP = 4;
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
     * Constants used by TargetAlignDrive
     */
    public static class TargetAlignMap {
        public static final double ANGLE_THRESHOLD = 1;
    }

    public static final XboxController CONTROLLER = new XboxController(0);

    // Intake stuff

    static {
        DriveMap.FRONT_LEFT_DRIVE_MOTOR.setInverted(false);
        DriveMap.BACK_LEFT_DRIVE_MOTOR.setInverted(true);
        DriveMap.FRONT_RIGHT_DRIVE_MOTOR.setInverted(false);
        DriveMap.BACK_RIGHT_DRIVE_MOTOR.setInverted(false);

        DriveMap.FRONT_LEFT_DRIVE_MOTOR.setNeutralMode(NeutralMode.Brake);
        DriveMap.BACK_LEFT_DRIVE_MOTOR.setNeutralMode(NeutralMode.Brake);
        DriveMap.FRONT_RIGHT_DRIVE_MOTOR.setNeutralMode(NeutralMode.Brake);
        DriveMap.BACK_RIGHT_DRIVE_MOTOR.setNeutralMode(NeutralMode.Brake);

        DriveMap.BACK_LEFT_ANGLE_MOTOR.setInverted(true);
        DriveMap.FRONT_LEFT_ANGLE_MOTOR.setInverted(true);
        DriveMap.BACK_RIGHT_ANGLE_MOTOR.setInverted(true);
        DriveMap.FRONT_RIGHT_ANGLE_MOTOR.setInverted(true);

        DriveMap.BACK_LEFT_ANGLE_MOTOR.setNeutralMode(NeutralMode.Brake);
        DriveMap.FRONT_LEFT_ANGLE_MOTOR.setNeutralMode(NeutralMode.Brake);
        DriveMap.BACK_RIGHT_ANGLE_MOTOR.setNeutralMode(NeutralMode.Brake);
        DriveMap.FRONT_RIGHT_ANGLE_MOTOR.setNeutralMode(NeutralMode.Brake);

        DriveMap.FRONT_LEFT_DRIVE_MOTOR.setSelectedSensorPosition(1); // setting to integrated sensor
        DriveMap.BACK_LEFT_DRIVE_MOTOR.setSelectedSensorPosition(1);
        DriveMap.FRONT_RIGHT_DRIVE_MOTOR.setSelectedSensorPosition(1);
        DriveMap.BACK_RIGHT_DRIVE_MOTOR.setSelectedSensorPosition(1);

        // Shooter
        ShooterMap.RIGHT_SHOOTER_FALCON.set(ControlMode.Follower, ShooterMap.LEFT_SHOOTER_FALCON.getDeviceID());
        ShooterMap.RIGHT_SHOOTER_FALCON.setInverted(InvertType.OpposeMaster);
        ShooterMap.BELT.setNeutralMode(NeutralMode.Brake);

        ShooterMap.LEFT_SHOOTER_FALCON.configFactoryDefault();
        // ShooterMap.RIGHT_SHOOTER_FALCON.configFactoryDefault();

        ShooterMap.LEFT_SHOOTER_FALCON.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
                kPIDLoopIdx, kTimeoutMs);
        // ShooterMap.RIGHT_SHOOTER_FALCON.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
        // kPIDLoopIdx,
        // kTimeoutMs);

        ShooterMap.LEFT_SHOOTER_FALCON.setSensorPhase(true);
        // ShooterMap.RIGHT_SHOOTER_FALCON.setSensorPhase(true);

        ShooterMap.LEFT_SHOOTER_FALCON.setSelectedSensorPosition(1);

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

        IntakeMap.ULTRASONIC.setAutomaticMode(true);
        IntakeMap.ULTRASONIC.setDistanceUnits(Ultrasonic.Unit.kInches);
        // IntakeMap.COLOR_SENSOR_MID.setAutomaticMode(true);
        // IntakeMap.COLOR_SENSOR_MID.setDistanceUnits(Ultrasonic.Unit.kInches);
    }
}
