package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shoot {

    Constants constants;
    double xSpeed;
    int loops;
    static double rotpm;
    WPI_TalonFX falcon;
    WPI_TalonFX falcon2;
    // sparks
    XboxController controller;
    WPI_TalonSRX belt; // belt talon

    public Shoot() {
        constants = new Constants();
        rotpm = 3800;
        // device IDs are completely arbitrary and NEED to be changed
        // spark tests
        falcon = RobotMap.shooter1; // GOOD + is right
        falcon2 = RobotMap.shooter2;
        falcon2.setInverted(true); // GET REAL PORT!
        controller = new XboxController(0);
        loops = 0;
        belt = new WPI_TalonSRX(11);
        belt.setNeutralMode(NeutralMode.Coast);

        falcon.configFactoryDefault();
        falcon2.configFactoryDefault();

        falcon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kPIDLoopIdx,
                Constants.kTimeoutMs);
        falcon2.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kPIDLoopIdx,
                Constants.kTimeoutMs);

        falcon.setSensorPhase(true);
        falcon2.setSensorPhase(true);

        falcon.setSelectedSensorPosition(0);

        /* Config the peak and nominal outputs */
        falcon.configNominalOutputForward(0, Constants.kTimeoutMs);
        falcon.configNominalOutputReverse(0, Constants.kTimeoutMs);
        falcon.configPeakOutputForward(1, Constants.kTimeoutMs);
        falcon.configPeakOutputReverse(-1, Constants.kTimeoutMs);

        /* Config the Velocity closed loop gains in slot0 */
        falcon.config_kF(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kF, Constants.kTimeoutMs);
        falcon.config_kP(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kP, Constants.kTimeoutMs);
        falcon.config_kI(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kI, Constants.kTimeoutMs);
        falcon.config_kD(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kD, Constants.kTimeoutMs);

        /* Config the peak and nominal outputs */
        falcon2.configNominalOutputForward(0, Constants.kTimeoutMs);
        falcon2.configNominalOutputReverse(0, Constants.kTimeoutMs);
        falcon2.configPeakOutputForward(1, Constants.kTimeoutMs);
        falcon2.configPeakOutputReverse(-1, Constants.kTimeoutMs);

        /* Config the Velocity closed loop gains in slot0 */
        falcon2.config_kF(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kF, Constants.kTimeoutMs);
        falcon2.config_kP(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kP, Constants.kTimeoutMs);
        falcon2.config_kI(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kI, Constants.kTimeoutMs);
        falcon2.config_kD(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kD, Constants.kTimeoutMs);

    }

    public void spin(double speed) {
        // double output = pid.calculate(spark.getEncoder().getVelocity());
        falcon.set(speed);
        falcon2.set(-speed);
    }

    public void smartDashboardOutput() {
        // falcon.getSelectedSensorPosition();
        SmartDashboard.putNumber("RPM", (falcon.getSelectedSensorVelocity(0)));
        System.out.println(falcon.getSelectedSensorVelocity(0));
    }

    public void drive() {
        falcon2.set(-xSpeed);
        falcon.set(xSpeed);

    }

    public void control() {
        double targetVelocity_UnitsPer100ms = constants.rpmToRotatPer100Mili(rotpm) * Constants.kEncoderUnitsPerRev;
        /* 500 RPM in either direction */
           
        /*IF TRIGGER PRESSED
            SPIN SHOOTER
            IF VELOCITY HIGH ENOUGH
                SPIN BELT
            ELSE
                DONT
        */
         
        if(RobotMap.controller.getTriggerAxis(Hand.kRight)>.6){
            falcon.set(ControlMode.Velocity, targetVelocity_UnitsPer100ms);
            falcon2.set(ControlMode.Velocity, targetVelocity_UnitsPer100ms);
            //belt.set(-.6);
        }
        else{
            falcon.set(0);
            falcon2.set(0);
        }
    }

}