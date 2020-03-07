package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.InvertType;
import com.ctre.phoenix.motorcontrol.NeutralMode;

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

    private Shooter() {  //TEST!!!!!
        RobotMap.ShooterMap.LEFT_SHOOTER_FALCON.configFactoryDefault();
		RobotMap.ShooterMap.RIGHT_SHOOTER_FALCON.configFactoryDefault();

		RobotMap.IntakeMap.BELT.setNeutralMode(NeutralMode.Brake);

		RobotMap.ShooterMap.LEFT_SHOOTER_FALCON.setInverted(false);
		RobotMap.ShooterMap.RIGHT_SHOOTER_FALCON.set(ControlMode.Follower, RobotMap.ShooterMap.LEFT_SHOOTER_FALCON.getDeviceID());
		RobotMap.ShooterMap.RIGHT_SHOOTER_FALCON.setInverted(InvertType.OpposeMaster);
		
		RobotMap.ShooterMap.LEFT_SHOOTER_FALCON.enableVoltageCompensation(true);
		RobotMap.ShooterMap.LEFT_SHOOTER_FALCON.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, RobotMap.kPIDLoopIdx,
        RobotMap.kTimeoutMs);
		RobotMap.ShooterMap.LEFT_SHOOTER_FALCON.setSensorPhase(true);
		//RobotMap.ShooterMap.LEFT_SHOOTER_FALCON.setSelectedSensorPosition(1);  THE INTENT WAS TO SET TO INTEGRATED SENSOR

		/* Config the peak and nominal outputs */
		RobotMap.ShooterMap.LEFT_SHOOTER_FALCON.configNominalOutputForward(0, RobotMap.kTimeoutMs);
		RobotMap.ShooterMap.LEFT_SHOOTER_FALCON.configNominalOutputReverse(0,  RobotMap.kTimeoutMs);
		RobotMap.ShooterMap.LEFT_SHOOTER_FALCON.configPeakOutputForward(1,  RobotMap.kTimeoutMs);
		RobotMap.ShooterMap.LEFT_SHOOTER_FALCON.configPeakOutputReverse(-1,  RobotMap.kTimeoutMs);

		/* Config the Velocity closed loop gains in slot0 */
		RobotMap.ShooterMap.LEFT_SHOOTER_FALCON.config_kF(RobotMap.kPIDLoopIdx, 0.055,  RobotMap.kTimeoutMs); // .45 *(1023.0/7200.0)
		RobotMap.ShooterMap.LEFT_SHOOTER_FALCON.config_kP(RobotMap.kPIDLoopIdx, 0.4,  RobotMap.kTimeoutMs);
		RobotMap.ShooterMap.LEFT_SHOOTER_FALCON.config_kI(RobotMap.kPIDLoopIdx, 0,  RobotMap.kTimeoutMs);
		RobotMap.ShooterMap.LEFT_SHOOTER_FALCON.config_kD(RobotMap.kPIDLoopIdx, 0,  RobotMap.kTimeoutMs);

		// Intake

		//IntakeMap.ULTRASONIC.setAutomaticMode(true);
		//IntakeMap.ULTRASONIC.setDistanceUnits(Ultrasonic.Unit.kInches);
		// IntakeMap.COLOR_SENSOR_MID.setAutomaticMode(true);
		// IntakeMap.COLOR_SENSOR_MID.setDistanceUnits(Ultrasonic.Unit.kInches);
       
       
        RobotMap.ShooterMap.LEFT_SHOOTER_FALCON.config_kF(RobotMap.ShooterMap.PID_LOOP_IDX,
            RobotMap.StateChooser.kF, RobotMap.ShooterMap.TIMEOUT_MS); // .45 *(1023.0/7200.0)

        RobotMap.ShooterMap.LEFT_SHOOTER_FALCON.config_kP(RobotMap.ShooterMap.PID_LOOP_IDX,
            RobotMap.StateChooser.kP, RobotMap.ShooterMap.TIMEOUT_MS);
        // rotpm = 4000;// 3800 - moved to RobotMap.ShooterMap.RPM
    }

    public void smartDashboardOutput() {
        // falcon.getSelectedSensorPosition();
        SmartDashboard.putNumber("RPM",
                (Math.abs(getRPM())));
        SmartDashboard.putNumber("Falcon Output", RobotMap.ShooterMap.LEFT_SHOOTER_FALCON.getMotorOutputPercent());
        // System.out.println(RobotMap.ShooterMap.LEFT_SHOOTER_FALCON.getSelectedSensorVelocity(0));
    }

    public void control() {
        double targetVelocity_UnitsPer100ms = rpmToRotatPer100Mili(RobotMap.StateChooser.RPM)/*RobotMap.ShooterMap.RPM)*/
                * RobotMap.ShooterMap.ENCODER_UNITS_PER_REV;
        /* 500 RPM in either direction */
        if (RobotMap.CONTROLLER.getTriggerAxis(Hand.kLeft) > .6 || RobotMap.AutoBooleans.SHOOT_NOW) {
            RobotMap.ShooterMap.LEFT_SHOOTER_FALCON.set(ControlMode.Velocity, -targetVelocity_UnitsPer100ms);
            // belt.set(-.6);
            SmartDashboard.putNumber("Shooter voltage L", RobotMap.ShooterMap.LEFT_SHOOTER_FALCON.get());
		    SmartDashboard.putNumber("Shooter voltage R", RobotMap.ShooterMap.RIGHT_SHOOTER_FALCON.get());
        } else {
            RobotMap.ShooterMap.LEFT_SHOOTER_FALCON.set(0);
        }
    }
    public void spinTest()
    {
        RobotMap.ShooterMap.LEFT_SHOOTER_FALCON.set(.8);
    }

    public static double rpmToRotatPer100Mili(double rpm) {
        return rpm / 600;
    }

    public static double sensorUnitsToRPM(double senUnits) {
        return senUnits * 600 / RobotMap.ShooterMap.ENCODER_UNITS_PER_REV;

    }
    public static double getRPM()
    {
        return sensorUnitsToRPM(RobotMap.ShooterMap.LEFT_SHOOTER_FALCON.getSelectedSensorVelocity(0));
    }

}
