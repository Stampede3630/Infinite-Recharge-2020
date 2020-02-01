// /*----------------------------------------------------------------------------*/
// /* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
// /* Open Source Software - may be modified and shared by FRC teams. The code   */
// /* must be accompanied by the FIRST BSD license file in the root directory of */
// /* the project.                                                               */
// /*----------------------------------------------------------------------------*/

// package frc.robot;

// import com.ctre.phoenix.motorcontrol.ControlMode;
// import com.ctre.phoenix.motorcontrol.FeedbackDevice;
// import com.ctre.phoenix.motorcontrol.NeutralMode;
// import com.ctre.phoenix.motorcontrol.can.TalonFXConfiguration;
// import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
// import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

// import edu.wpi.first.wpilibj.Talon;
// import edu.wpi.first.wpilibj.XboxController;
// import edu.wpi.first.wpilibj.GenericHID.Hand;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

// public class Shooter {

//     Constants constants;
//     double xSpeed;
//     int loops;
//     double rotpm;
//     // sparks
//     private WPI_TalonFX falcon;
//     private WPI_TalonFX falcon2;
//     XboxController controller;
//     WPI_TalonSRX bird; 
    

//     public Shooter() {
//         constants = new Constants();
//         rotpm = 4000;
//         // device IDs are completely arbitrary and NEED to be changed
//         // spark tests
//         falcon = new WPI_TalonFX(12); //GOOD + is right
//         falcon2 = new WPI_TalonFX(13);
//         falcon2.setInverted(true); // GET REAL PORT!
//         controller = new XboxController(0);
//         loops = 0;
//         bird = new WPI_TalonSRX(11);
        
//         bird.setNeutralMode(NeutralMode.Coast);
//         falcon.setNeutralMode(NeutralMode.Coast);
//         falcon2.setNeutralMode(NeutralMode.Coast);

//         falcon.configFactoryDefault();
//         falcon2.configFactoryDefault();
//         falcon.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, Constants.kPIDLoopIdx, Constants.kTimeoutMs);
//         falcon2.configSelectedFeedbackSensor(FeedbackDevice.IntegratedSensor, Constants.kPIDLoopIdx, Constants.kTimeoutMs);

//         falcon.setSensorPhase(true);
//         falcon2.setSensorPhase(true);

//         falcon.setSelectedSensorPosition(0);

//         /* Config the peak and nominal outputs */
//         falcon.configNominalOutputForward(0, Constants.kTimeoutMs);
// 		falcon.configNominalOutputReverse(0, Constants.kTimeoutMs);
// 		falcon.configPeakOutputForward(1, Constants.kTimeoutMs);
// 		falcon.configPeakOutputReverse(-1, Constants.kTimeoutMs);

// 		/* Config the Velocity closed loop gains in slot0 */
// 		falcon.config_kF(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kF, Constants.kTimeoutMs);
// 		falcon.config_kP(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kP, Constants.kTimeoutMs);
// 		falcon.config_kI(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kI, Constants.kTimeoutMs);
//         falcon.config_kD(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kD, Constants.kTimeoutMs);
        
//         /* Config the peak and nominal outputs */
//         falcon2.configNominalOutputForward(0, Constants.kTimeoutMs);
// 		falcon2.configNominalOutputReverse(0, Constants.kTimeoutMs);
// 		falcon2.configPeakOutputForward(1, Constants.kTimeoutMs);
// 		falcon2.configPeakOutputReverse(-1, Constants.kTimeoutMs);

// 		/* Config the Velocity closed loop gains in slot0 */
// 		falcon2.config_kF(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kF, Constants.kTimeoutMs);
// 		falcon2.config_kP(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kP, Constants.kTimeoutMs);
// 		falcon2.config_kI(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kI, Constants.kTimeoutMs);
// 		falcon2.config_kD(Constants.kPIDLoopIdx, Constants.kGains_Velocit.kD, Constants.kTimeoutMs);


//     }

//     public void spin() {
//         // double output = pid.calculate(spark.getEncoder().getVelocity());
//         falcon.set(.75);
//         falcon2.set(-.75);
//     }

//     public void smartDashboardOutput() {
//         //falcon.getSelectedSensorPosition();
//         SmartDashboard.putNumber("RPM", (constants.sensorUnitsToRPM(falcon.getSelectedSensorVelocity(0))));
//         //System.out.println(falcon.getSelectedSensorVelocity(0));
//         SmartDashboard.putNumber("Motor output percent", falcon.getMotorOutputPercent());
//     }

//     public void drive() {
//         falcon2.set(0);
//         falcon.set(0);

//     }

//     public void control(){
//     double targetVelocity_UnitsPer100ms = constants.rpmToRotatPer100Mili(rotpm) * Constants.kEncoderUnitsPerRev;
//       /* 500 RPM in either direction */
//       falcon.set(ControlMode.Velocity, targetVelocity_UnitsPer100ms);
//       falcon2.set(ControlMode.Velocity, targetVelocity_UnitsPer100ms);
//       //bird.set(-.7);
//       //falcon.set(0.8);
//       //falcon2.set(0.8);
//     }

// }