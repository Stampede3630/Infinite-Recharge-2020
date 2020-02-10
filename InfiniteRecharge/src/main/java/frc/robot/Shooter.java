package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter {

    double xSpeed;
    static double rotpm;
    WPI_TalonFX leftShooterFalcon;
    WPI_TalonFX rightShooterFalcon;
    // sparks
    XboxController controller;
    WPI_TalonSRX belt; // belt talon

    public Shooter() {
        rotpm = 4000;// 3800
        leftShooterFalcon = RobotMap.leftShooterFalcon; // GOOD + is right
        rightShooterFalcon = RobotMap.rightShooterFalcon;

    }

    public void smartDashboardOutput() {
        // falcon.getSelectedSensorPosition();
        SmartDashboard.putNumber("RPM", (sensorUnitsToRPM(leftShooterFalcon.getSelectedSensorVelocity(1))));
        SmartDashboard.putNumber("Falcon Output", leftShooterFalcon.getMotorOutputPercent());
        System.out.println(leftShooterFalcon.getSelectedSensorVelocity(0));
    }

    public void control() {
        double targetVelocity_UnitsPer100ms = rpmToRotatPer100Mili(rotpm) * kEncoderUnitsPerRev;
        /* 500 RPM in either direction */
        leftShooterFalcon.config_kF(RobotMap.kPIDLoopIdx, SmartDashboard.getNumber("kF", 0), RobotMap.kTimeoutMs); // .45 *(1023.0/7200.0)
        leftShooterFalcon.config_kP(RobotMap.kPIDLoopIdx, SmartDashboard.getNumber("kP", 0), RobotMap.kTimeoutMs);
        leftShooterFalcon.config_kI(RobotMap.kPIDLoopIdx, SmartDashboard.getNumber("kI", 0), RobotMap.kTimeoutMs);
        leftShooterFalcon.config_kD(RobotMap.kPIDLoopIdx, SmartDashboard.getNumber("kD", 0), RobotMap.kTimeoutMs);

        if (RobotMap.controller.getTriggerAxis(Hand.kLeft) > .6) {
            leftShooterFalcon.set(ControlMode.Velocity, targetVelocity_UnitsPer100ms);
            // rightShooterFalcon.set(ControlMode.Velocity, targetVelocity_UnitsPer100ms);
            // belt.set(-.6);
        } else {
            leftShooterFalcon.set(0);
            // rightShooterFalcon.set(0);
        }
    }

    /**
     * Which PID slot to pull gains from. Starting 2018, you can choose from 0,1,2
     * or 3. Only the first two (0,1) are visible in web-based configuration.
     */
    public static final int kSlotIdx = 0;

    public static final double kEncoderUnitsPerRev = 2048;// 4096;

    public static double rpmToRotatPer100Mili(double rpm) {
        double milliSec = rpm / 600;
        return milliSec;
    }

    public static double sensorUnitsToRPM(double senUnits) {
        return senUnits * 600 / kEncoderUnitsPerRev;

    }

}
