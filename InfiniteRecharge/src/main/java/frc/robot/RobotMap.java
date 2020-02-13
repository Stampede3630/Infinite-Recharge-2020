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

/**
 * Add your docs here.
 */
public class RobotMap {
    //general stuff
    public static XboxController controller = new XboxController(0);
    public static final AHRS m_gyro = new AHRS(SPI.Port.kMXP);



    // climbing stuff
    public static CANSparkMax trolleySpark = new CANSparkMax(14, MotorType.kBrushless);
    public static CANSparkMax elevatorSpark = new CANSparkMax(8, MotorType.kBrushless);
    public static DigitalInput maxLimitSwitch = new DigitalInput(20);
    public static DigitalInput minLimitSwitch = new DigitalInput(19);
    /*
     * public static DigitalInput elevatorMaxExtension = new DigitalInput(20);
     * public static DigitalInput elevatorMinExtension = new DigitalInput(19);
     */



    //shooter stuff
    public static WPI_TalonFX leftShooterFalcon = new WPI_TalonFX(12);
    public static WPI_TalonFX rightShooterFalcon = new WPI_TalonFX(13);
    public static final WPI_TalonSRX belt;
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



    // Swerve hardware
    public static final WPI_VictorSPX DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR = new WPI_VictorSPX(2);
    public static final WPI_TalonFX DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR = new WPI_TalonFX(1);
    public static final AnalogInput DRIVETRAIN_FRONT_RIGHT_ANGLE_ENCODER = new AnalogInput(0);
    public static final double FRONT_RIGHT_ANGLE_OFFSET = 2.607 + Math.PI;

    public static final WPI_VictorSPX DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR = new WPI_VictorSPX(6);
    public static final WPI_TalonFX DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR = new WPI_TalonFX(5);
    public static final AnalogInput DRIVETRAIN_BACK_LEFT_ANGLE_ENCODER = new AnalogInput(1);
    public static final double BACK_LEFT_ANGLE_OFFSET = -0.339;

    public static final WPI_VictorSPX DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR = new WPI_VictorSPX(4);
    public static final WPI_TalonFX DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR = new WPI_TalonFX(3);
    public static final AnalogInput DRIVETRAIN_BACK_RIGHT_ANGLE_ENCODER = new AnalogInput(2);
    public static final double BACK_RIGHT_ANGLE_OFFSET = -1.596 + Math.PI;

    public static final WPI_VictorSPX DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR = new WPI_VictorSPX(8);
    public static final WPI_TalonFX DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR = new WPI_TalonFX(7);
    public static final AnalogInput DRIVETRAIN_FRONT_LEFT_ANGLE_ENCODER = new AnalogInput(3);
    public static final double FRONT_LEFT_ANGLE_OFFSET = 0.364; // radians



    // PID Constants/Contraints
    public static final double kMaxSpeed = 4; // 3 meters per second
    public static final double kMaxAngularSpeed = Math.PI;



    //Intake stuff
    public static CANSparkMax intakeWheels;// first spin wheel
    public static WPI_TalonSRX pinwheel; // from box to belt
    public static DoubleSolenoid armsSolenoid; // lowers the arms
    public static Ultrasonic ultrasonic; // on the ground of the belt box
    public static ColorSensorV3 colorSensorMid;
    public static ColorSensorV3 colorSensorHigh;

    static {
        DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR.setInverted(true);
        DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR.setInverted(false);
        DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR.setInverted(false);
        DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR.setInverted(false);

        DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR.setNeutralMode(NeutralMode.Brake);
        DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR.setNeutralMode(NeutralMode.Brake);
        DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR.setNeutralMode(NeutralMode.Brake);
        DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR.setNeutralMode(NeutralMode.Brake);

        DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR.setInverted(true);
        DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR.setInverted(true);
        DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR.setInverted(true);
        DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR.setInverted(true);

        DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR.setNeutralMode(NeutralMode.Brake);
        DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR.setNeutralMode(NeutralMode.Brake);
        DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR.setNeutralMode(NeutralMode.Brake);
        DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR.setNeutralMode(NeutralMode.Brake);

        DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR.setSelectedSensorPosition(1); // setting to integrated sensor
        DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR.setSelectedSensorPosition(1);
        DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR.setSelectedSensorPosition(1);
        DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR.setSelectedSensorPosition(1);

        

        //Shooter
        rightShooterFalcon.set(ControlMode.Follower, RobotMap.leftShooterFalcon.getDeviceID());
        rightShooterFalcon.setInverted(InvertType.OpposeMaster);
        belt = new WPI_TalonSRX(11);
        belt.setNeutralMode(NeutralMode.Brake);

        leftShooterFalcon.configFactoryDefault();
        // rightShooterFalcon.configFactoryDefault();

        leftShooterFalcon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, kPIDLoopIdx,
                kTimeoutMs);
        // rightShooterFalcon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative,
        // kPIDLoopIdx,
        // kTimeoutMs);

        leftShooterFalcon.setSensorPhase(true);
        // rightShooterFalcon.setSensorPhase(true);

        leftShooterFalcon.setSelectedSensorPosition(1);

        /* Config the peak and nominal outputs */
        leftShooterFalcon.configNominalOutputForward(0, kTimeoutMs);
        leftShooterFalcon.configNominalOutputReverse(0, kTimeoutMs);
        leftShooterFalcon.configPeakOutputForward(1, kTimeoutMs);
        leftShooterFalcon.configPeakOutputReverse(-1, kTimeoutMs);

        /* Config the Velocity closed loop gains in slot0 */
        leftShooterFalcon.config_kF(kPIDLoopIdx, 0.055, kTimeoutMs); // .45 *(1023.0/7200.0)
        leftShooterFalcon.config_kP(kPIDLoopIdx, 0.4, kTimeoutMs);
        leftShooterFalcon.config_kI(kPIDLoopIdx, 0, kTimeoutMs);
        leftShooterFalcon.config_kD(kPIDLoopIdx, 0, kTimeoutMs);



        // Intake

        intakeWheels = new CANSparkMax(9, MotorType.kBrushless);
        armsSolenoid = new DoubleSolenoid(2, 3); // 2 solenoid on r
        colorSensorHigh = new ColorSensorV3(I2C.Port.kMXP);
        pinwheel = new WPI_TalonSRX(10);
        ultrasonic = new Ultrasonic(5, 3);
        colorSensorMid = new ColorSensorV3(I2C.Port.kOnboard);
        ultrasonic.setAutomaticMode(true);
        ultrasonic.setDistanceUnits(Ultrasonic.Unit.kInches);
        // colorSensorMid.setAutomaticMode(true);
        // colorSensorMid.setDistanceUnits(Ultrasonic.Unit.kInches);
    }
    /*
     * private static RobotMap thisInstance;
     * 
     * 
     * private RobotMap() { DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR.setInverted(true);
     * DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR.setInverted(false);
     * DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR.setInverted(false);
     * DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR.setInverted(false);
     * 
     * DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR.setNeutralMode(NeutralMode.Brake);
     * DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR.setNeutralMode(NeutralMode.Brake);
     * DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR.setNeutralMode(NeutralMode.Brake);
     * DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR.setNeutralMode(NeutralMode.Brake);
     * 
     * DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR.setInverted(true);
     * DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR.setInverted(true);
     * DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR.setInverted(true);
     * DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR.setInverted(true);
     * 
     * DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR.setSelectedSensorPosition(0);
     * DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR.setSelectedSensorPosition(0);
     * DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR.setSelectedSensorPosition(0);
     * DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR.setSelectedSensorPosition(0);
     * 
     * }
     * 
     * public static RobotMap getInstance() { if (thisInstance == null) {
     * thisInstance = new RobotMap();
     * 
     * } return thisInstance; }
     */
}
