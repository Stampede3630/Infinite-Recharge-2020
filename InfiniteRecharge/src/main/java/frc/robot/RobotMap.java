/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;

/**
 * Add your docs here.
 */
public class RobotMap {

    //climbing stuff, needs real ports
    public static CANSparkMax trolleySpark = new CANSparkMax(14, MotorType.kBrushless);
    public static CANSparkMax elevatorSpark = new CANSparkMax(8, MotorType.kBrushless);

    public static WPI_TalonFX leftShooterFalcon = new WPI_TalonFX(12);
    public static WPI_TalonFX rightShooterFalcon = new WPI_TalonFX(13);

    /*
    public static DigitalInput elevatorMaxExtension = new DigitalInput(20);
    public static DigitalInput elevatorMinExtension = new DigitalInput(19);
    */

    public static XboxController controller = new XboxController(0);

    public static final AHRS m_gyro = new AHRS(SPI.Port.kMXP);
    
    // Swerve hardware
    public static final WPI_TalonSRX DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR = new WPI_TalonSRX(2);
    public static final WPI_TalonFX DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR = new WPI_TalonFX(1);
    public static final AnalogInput DRIVETRAIN_FRONT_RIGHT_ANGLE_ENCODER = new AnalogInput(0);
    public static final double FRONT_RIGHT_ANGLE_OFFSET = 2.607 + Math.PI;

    public static final WPI_TalonSRX DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR = new WPI_TalonSRX(6);
    public static final WPI_TalonFX DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR = new WPI_TalonFX(5);
    public static final AnalogInput DRIVETRAIN_BACK_LEFT_ANGLE_ENCODER = new AnalogInput(1);
    public static final double BACK_LEFT_ANGLE_OFFSET = -0.339;

    public static final WPI_TalonSRX DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR = new WPI_TalonSRX(4);
    public static final WPI_TalonFX DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR = new WPI_TalonFX(3);
    public static final AnalogInput DRIVETRAIN_BACK_RIGHT_ANGLE_ENCODER = new AnalogInput(2);
    public static final double BACK_RIGHT_ANGLE_OFFSET = -1.596 + Math.PI;

    public static final WPI_TalonSRX DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR = new WPI_TalonSRX(8);
    public static final WPI_TalonFX DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR = new WPI_TalonFX(7);
    public static final AnalogInput DRIVETRAIN_FRONT_LEFT_ANGLE_ENCODER = new AnalogInput(3);
    public static final double FRONT_LEFT_ANGLE_OFFSET = 0.364; // radians

    //PID Constants/Contraints
    public static final double kMaxSpeed = 4; // 3 meters per second
    public static final double kMaxAngularSpeed = Math.PI;
    
    static
    {
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

        DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR.setSelectedSensorPosition(1); //setting to integrated sensor
        DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR.setSelectedSensorPosition(1);
        DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR.setSelectedSensorPosition(1);
        DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR.setSelectedSensorPosition(1);

    }
    /*
    private static RobotMap thisInstance;


    private RobotMap() {
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

        DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR.setSelectedSensorPosition(0);
        DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR.setSelectedSensorPosition(0);
        DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR.setSelectedSensorPosition(0);
        DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR.setSelectedSensorPosition(0);

    }

    public static RobotMap getInstance() {
        if (thisInstance == null) {
            thisInstance = new RobotMap();

        }
        return thisInstance;
    }
    */
}
