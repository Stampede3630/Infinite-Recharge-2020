package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shooter {

    private static Shooter instance;

    static {
        instance = new Shooter();
    }

    public static Shooter getInstance() {
        return instance;
    }

    private Shooter() { // TEST!!!!!
        RobotMap.ShooterMap.LEFT_SHOOTER_FALCON.config_kF(RobotMap.ShooterMap.PID_LOOP_IDX, RobotMap.StateChooser.kF,
                RobotMap.ShooterMap.TIMEOUT_MS); // .45 *(1023.0/7200.0)

        RobotMap.ShooterMap.LEFT_SHOOTER_FALCON.config_kP(RobotMap.ShooterMap.PID_LOOP_IDX, RobotMap.StateChooser.kP,
                RobotMap.ShooterMap.TIMEOUT_MS);
        // rotpm = 4000;// 3800 - moved to RobotMap.ShooterMap.RPM
    }

    public void smartDashboardOutput() {
        // falcon.getSelectedSensorPosition();
        SmartDashboard.putNumber("RPM", (Math.abs(getRPM())));
        SmartDashboard.putNumber("Falcon Output", RobotMap.ShooterMap.LEFT_SHOOTER_FALCON.getMotorOutputPercent());
        // System.out.println(RobotMap.ShooterMap.LEFT_SHOOTER_FALCON.getSelectedSensorVelocity(0));
    }

    public void control() {
        double targetVelocity_UnitsPer100ms = rpmToRotatPer100Mili(RobotMap.StateChooser.RPM)/*
                                                                                              * RobotMap.ShooterMap.RPM)
                                                                                              */
                * RobotMap.ShooterMap.ENCODER_UNITS_PER_REV;

        /* 500 RPM in either direction */
        if (RobotMap.CONTROLLER.getTriggerAxis(Hand.kLeft) > .6 || RobotMap.AutoBooleans.SHOOT_NOW) {

            RobotMap.ShooterMap.LEFT_SHOOTER_FALCON.set(ControlMode.Velocity, -targetVelocity_UnitsPer100ms);
            // belt.set(-.6);
            System.out.println("******************************SHOOTER SHOULD BE SHOOTING");
        } else {
            RobotMap.ShooterMap.LEFT_SHOOTER_FALCON.set(0);
        }
    }

    public static double rpmToRotatPer100Mili(double rpm) {
        return rpm / 600;
    }

    public static double sensorUnitsToRPM(double senUnits) {
        return senUnits * 600 / RobotMap.ShooterMap.ENCODER_UNITS_PER_REV;

    }

    public static double getRPM() {
        return sensorUnitsToRPM(RobotMap.ShooterMap.LEFT_SHOOTER_FALCON.getSelectedSensorVelocity(0));
    }

}
