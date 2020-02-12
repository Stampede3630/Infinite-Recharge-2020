/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

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
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

/**
 * Add your docs here.
 */
public class RobotMap {

    public static class AutoConstants { // Is this being used?
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

    public static class ClimbMap {
        // Climbing stuff
        public static final CANSparkMax TROLLEY_SPARK = new CANSparkMax(14, MotorType.kBrushless);
        public static final CANSparkMax ELEVATOR_SPARK = new CANSparkMax(8, MotorType.kBrushless);
        public static final DigitalInput MAX_LIMIT_SWITCH = new DigitalInput(20);
        public static final DigitalInput MIN_LIMIT_SWITCH = new DigitalInput(19);
    }

    public static class ShooterMap {
        // Shooter stuff
        public static final WPI_TalonFX LEFT_SHOOTER_FALCON = new WPI_TalonFX(12);
        public static final WPI_TalonFX RIGHT_SHOOTER_FALCON = new WPI_TalonFX(13);
    }

    /*
     * public static DigitalInput elevatorMaxExtension = new DigitalInput(20);
     * public static DigitalInput elevatorMinExtension = new DigitalInput(19);
     */

    public static class SensorMap {
        public static final AHRS GYRO = new AHRS(SPI.Port.kMXP);
    }

    public static class DriveMap {
        // Swerve hardware
        public static final WPI_TalonSRX FRONT_RIGHT_ANGLE_MOTOR = new WPI_TalonSRX(2);
        public static final WPI_TalonFX FRONT_RIGHT_DRIVE_MOTOR = new WPI_TalonFX(1);
        public static final AnalogInput FRONT_RIGHT_ANGLE_ENCODER = new AnalogInput(0);
        public static final double FRONT_RIGHT_ANGLE_OFFSET = 2.607 + Math.PI;

        public static final WPI_TalonSRX BACK_LEFT_ANGLE_MOTOR = new WPI_TalonSRX(6);
        public static final WPI_TalonFX BACK_LEFT_DRIVE_MOTOR = new WPI_TalonFX(5);
        public static final AnalogInput BACK_LEFT_ANGLE_ENCODER = new AnalogInput(1);
        public static final double BACK_LEFT_ANGLE_OFFSET = -0.339;

        public static final WPI_TalonSRX BACK_RIGHT_ANGLE_MOTOR = new WPI_TalonSRX(4);
        public static final WPI_TalonFX BACK_RIGHT_DRIVE_MOTOR = new WPI_TalonFX(3);
        public static final AnalogInput BACK_RIGHT_ANGLE_ENCODER = new AnalogInput(2);
        public static final double BACK_RIGHT_ANGLE_OFFSET = -1.596 + Math.PI;

        public static final WPI_TalonSRX FRONT_LEFT_ANGLE_MOTOR = new WPI_TalonSRX(8);
        public static final WPI_TalonFX FRONT_LEFT_DRIVE_MOTOR = new WPI_TalonFX(7);
        public static final AnalogInput FRONT_LEFT_ANGLE_ENCODER = new AnalogInput(3);
        public static final double FRONT_LEFT_ANGLE_OFFSET = 0.364; // radians
    }

    public static class PIDConstraints {
        // PID Constants/Contraints
        public static final double MAX_SPEED = 4; // 3 meters per second
        public static final double MAX_ANGULAR_SPEED = Math.PI;
    }

    public static class Pipelines {
        public static final int BALL_FOLLOW = 1;
        public static final int TARGET_LINEUP = 4;
    }

    public static class BallFollowMap {

        public static final int FLICKER_PROTECTION = 4;
        public static final double FAST_SEARCH = 0.9;
        public static final double SLOW_SEARCH = 0.6;
        public static final double FOLLOW_SPEED_MULTIPLIER = 0.7;

        public static final double VELOCITY_MODIFIER_MULT = 0.5;
    }

    public static final XboxController CONTROLLER = new XboxController(0);

    static {
        DriveMap.FRONT_LEFT_DRIVE_MOTOR.setInverted(true);
        DriveMap.BACK_LEFT_DRIVE_MOTOR.setInverted(false);
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

    }
}
