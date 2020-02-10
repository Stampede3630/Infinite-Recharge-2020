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

    double xSpeed;
    static double rotpm;
    WPI_TalonFX leftShooterFalcon;
    WPI_TalonFX rightShooterFalcon;
    // sparks
    XboxController controller;
    WPI_TalonSRX belt; // belt talon

    public Shooter() {
        rotpm = 4000;//3800
        // device IDs are completely arbitrary and NEED to be changed
        // spark tests
        leftShooterFalcon = RobotMap.leftShooterFalcon; // GOOD + is right
        rightShooterFalcon = RobotMap.rightShooterFalcon;
        rightShooterFalcon.set(ControlMode.Follower, RobotMap.leftShooterFalcon.getDeviceID());
        rightShooterFalcon.setInverted(InvertType.OpposeMaster); 
        controller = new XboxController(0);
        belt = new WPI_TalonSRX(11);
        belt.setNeutralMode(NeutralMode.Coast);

        leftShooterFalcon.configFactoryDefault();
        // rightShooterFalcon.configFactoryDefault();

        leftShooterFalcon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, kPIDLoopIdx,
                kTimeoutMs);
        // rightShooterFalcon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, kPIDLoopIdx,
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
        leftShooterFalcon.config_kF(kPIDLoopIdx,0.055, kTimeoutMs); //.45 *(1023.0/7200.0)
        leftShooterFalcon.config_kP(kPIDLoopIdx, 0.4, kTimeoutMs);
        leftShooterFalcon.config_kI(kPIDLoopIdx, 0, kTimeoutMs);
        leftShooterFalcon.config_kD(kPIDLoopIdx, 0, kTimeoutMs);

        /* Config the peak and nominal outputs */
        // rightShooterFalcon.configNominalOutputForward(0, kTimeoutMs);
        // rightShooterFalcon.configNominalOutputReverse(0, kTimeoutMs);
        // rightShooterFalcon.configPeakOutputForward(1, kTimeoutMs);
        // rightShooterFalcon.configPeakOutputReverse(-1, kTimeoutMs);

        /* Config the Velocity closed loop gains in slot0 */
        // rightShooterFalcon.config_kF(kPIDLoopIdx, kGains_Velocit.kF, kTimeoutMs);
        // rightShooterFalcon.config_kP(kPIDLoopIdx, kGains_Velocit.kP, kTimeoutMs);
        // rightShooterFalcon.config_kI(kPIDLoopIdx, kGains_Velocit.kI, kTimeoutMs);
        // rightShooterFalcon.config_kD(kPIDLoopIdx, kGains_Velocit.kD, kTimeoutMs);

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
           
        /*IF TRIGGER PRESSED
            SPIN SHOOTER
            IF VELOCITY HIGH ENOUGH
                SPIN BELT
            ELSE
                DONT
        */
        leftShooterFalcon.config_kF(kPIDLoopIdx,SmartDashboard.getNumber("kF", 0), kTimeoutMs); //.45 *(1023.0/7200.0)
        leftShooterFalcon.config_kP(kPIDLoopIdx, SmartDashboard.getNumber("kP", 0), kTimeoutMs);
        leftShooterFalcon.config_kI(kPIDLoopIdx, SmartDashboard.getNumber("kI", 0), kTimeoutMs);
        leftShooterFalcon.config_kD(kPIDLoopIdx, SmartDashboard.getNumber("kD", 0), kTimeoutMs);

        if(RobotMap.controller.getTriggerAxis(Hand.kLeft)>.6){
            leftShooterFalcon.set(ControlMode.Velocity, targetVelocity_UnitsPer100ms);
            // rightShooterFalcon.set(ControlMode.Velocity, targetVelocity_UnitsPer100ms);
            //belt.set(-.6);
        }
        else{
            leftShooterFalcon.set(0);
            // rightShooterFalcon.set(0);
        }
    }
	/**
	 * Which PID slot to pull gains from. Starting 2018, you can choose from
	 * 0,1,2 or 3. Only the first two (0,1) are visible in web-based
	 * configuration.
	 */
	public static final int kSlotIdx = 0;

	/**
	 * Talon SRX/ Victor SPX will supported multiple (cascaded) PID loops. For
	 * now we just want the primary one.
	 */
	public static final int kPIDLoopIdx = 0;

	/**
	 * Set to zero to skip waiting for confirmation, set to nonzero to wait and
	 * report to DS if action fails.
	 */
    public static final int kTimeoutMs = 30;

    public static final double kEncoderUnitsPerRev = 2048;//4096;

    public static double rpmToRotatPer100Mili(double rpm){
        double milliSec = rpm/600;
        return milliSec;
	}
	
	public static double sensorUnitsToRPM(double senUnits)
	{
		return senUnits * 600 / kEncoderUnitsPerRev;

	}

}
