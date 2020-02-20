package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.XboxController;
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
    
    private double xSpeed;
    private double rotpm;
    private Shooter() {
        rotpm = 4000;// 3800
    }

    public void smartDashboardOutput() {
        // falcon.getSelectedSensorPosition();
        SmartDashboard.putNumber("RPM", (sensorUnitsToRPM(RobotMap.ShooterMap.LEFT_SHOOTER_FALCON.getSelectedSensorVelocity(1))));
        SmartDashboard.putNumber("Falcon Output", RobotMap.ShooterMap.LEFT_SHOOTER_FALCON.getMotorOutputPercent());
        //System.out.println(RobotMap.ShooterMap.LEFT_SHOOTER_FALCON.getSelectedSensorVelocity(0));
    }

    public void control() {
        double targetVelocity_UnitsPer100ms = rpmToRotatPer100Mili(rotpm) * kEncoderUnitsPerRev;
        /* 500 RPM in either direction */
        RobotMap.ShooterMap.LEFT_SHOOTER_FALCON.config_kF(kPIDLoopIdx, SmartDashboard.getNumber("kF", 0.055), kTimeoutMs); // .45 *(1023.0/7200.0)
        RobotMap.ShooterMap.LEFT_SHOOTER_FALCON.config_kP(kPIDLoopIdx, SmartDashboard.getNumber("kP", 1), kTimeoutMs);
        RobotMap.ShooterMap.LEFT_SHOOTER_FALCON.config_kI(kPIDLoopIdx, SmartDashboard.getNumber("kI", 0), kTimeoutMs);
        RobotMap.ShooterMap.LEFT_SHOOTER_FALCON.config_kD(kPIDLoopIdx, SmartDashboard.getNumber("kD", 0), kTimeoutMs);
        
        if (RobotMap.CONTROLLER.getTriggerAxis(Hand.kLeft) > .6) {
            RobotMap.ShooterMap.LEFT_SHOOTER_FALCON.set(ControlMode.Velocity, -targetVelocity_UnitsPer100ms);
            // belt.set(-.6);
        } else {
            RobotMap.ShooterMap.LEFT_SHOOTER_FALCON.set(0);
        }
    }

    /**
     * Which PID slot to pull gains from. Starting 2018, you can choose from 0,1,2
     * or 3. Only the first two (0,1) are visible in web-based configuration.
     */
    public static final int kSlotIdx = 0;

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

    public static final double kEncoderUnitsPerRev = 2048;// 4096;

    public static double rpmToRotatPer100Mili(double rpm) {
        double milliSec = rpm / 600;
        return milliSec;
    }

    public static double sensorUnitsToRPM(double senUnits) {
        return senUnits * 600 / kEncoderUnitsPerRev;

    }

}
