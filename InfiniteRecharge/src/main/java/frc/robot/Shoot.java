package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.FeedbackDevice;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Shoot {

    Constants constants;
    double xSpeed;
    int loops;
    double rotpm;
    // sparks
    private WPI_TalonFX falcon;
    private WPI_TalonFX falcon2;
    XboxController controller;
    WPI_TalonSRX belt; //belt talon
    

    public Shoot() {
        constants = new Constants();
        rotpm = 3800;
        // device IDs are completely arbitrary and NEED to be changed
        // spark tests
        falcon = new WPI_TalonFX(12); //GOOD + is right
        falcon2 = new WPI_TalonFX(13);
        falcon2.setInverted(true); // GET REAL PORT!
        controller = new XboxController(0);
        loops = 0;
        belt = new WPI_TalonSRX(11);
        belt.setNeutralMode(NeutralMode.Coast);

        falcon.configFactoryDefault();
        falcon2.configFactoryDefault();

        falcon.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
        falcon2.configSelectedFeedbackSensor(FeedbackDevice.CTRE_MagEncoder_Relative, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

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

    public void spin() {
        // double output = pid.calculate(spark.getEncoder().getVelocity());
        falcon.set(.75);
        falcon2.set(-.75);
    }

    public void smartDashboardOutput() {
        //falcon.getSelectedSensorPosition();
        SmartDashboard.putNumber("RPM", (Constants.sensorUnitsToRPM(falcon.getSelectedSensorVelocity(1))));
        System.out.println(falcon.getSelectedSensorVelocity(0));
    }

    public void drive() {
        falcon2.set(-xSpeed);
        falcon.set(xSpeed);

    }

    public void control(){
        falcon.config_kF(Constants.kPIDLoopIdx, Robot.kF, Constants.kTimeoutMs);
        falcon2.config_kF(Constants.kPIDLoopIdx, Robot.kF, Constants.kTimeoutMs);
        System.out.println("`````````````````````"+Robot.kF);
    double targetVelocity_UnitsPer100ms = constants.rpmToRotatPer100Mili(rotpm) * Constants.kEncoderUnitsPerRev;
      /* 500 RPM in either direction */
     falcon.set(ControlMode.Velocity, targetVelocity_UnitsPer100ms);
      falcon2.set(ControlMode.Velocity, targetVelocity_UnitsPer100ms);
      //belt.set(-.6);
    }
    public void stop()
    {
        falcon.set(0);
        falcon2.set(0);
        //belt.set(0);
    }

}