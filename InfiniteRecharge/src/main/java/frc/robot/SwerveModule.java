/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.ctre.phoenix.motorcontrol.can.WPI_VictorSPX;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.kauailabs.navx.frc.AHRS;

public class SwerveModule {
  private static final double kWheelRadius = 0.051;
  private static final int kEncoderResolution = 2048;

  private static double kModuleMaxAngularVelocity = Math.PI * 2 * 2.6;
  private static double kModuleMaxAngularAcceleration = 6 * Math.PI; // radians per second squared //4

  public final WPI_TalonFX m_driveMotor;
  private final WPI_VictorSPX m_turningMotor;
  private double m_steeringAngle;
  private int m_driveScalar = 1;
  private final AnalogInput m_turningEncoder;
  double kPSpecial;
  double kD;
  AHRS currentAngle;

   double driveOutput;

  private final PIDController m_drivePIDController = new PIDController(0.1, 0, 0);

  private final ProfiledPIDController m_turningPIDController;// = new ProfiledPIDController(1, 0.0, 0.02, new
                                                             // TrapezoidProfile.Constraints(kModuleMaxAngularVelocity,
  // kModuleMaxAngularAcceleration));//0.9 (.05 / Math.PI)

  // new TrapezoidProfile.Constraints(kModuleMaxAngularVelocity,
  // kModuleMaxAngularAcceleration)
  // front left -(0,1) - PID: 0.8, 0.05
  // (4,5) -PID: 0.8, 0.05
  private final double angleOffset;

  /**
   * Constructs a SwerveModule.
   *
   * @param drivetrainFrontLeftDriveMotor ID for the drive motor.
   * @param drivetrainFrontLeftAngleMotor ID for the turning motor.
   */

  public SwerveModule(WPI_TalonFX drivetrainFrontLeftDriveMotor, WPI_VictorSPX drivetrainFrontLeftAngleMotor,
      AnalogInput drivetrainFrontLeftAngleEncoder, double angleChange) {
    if (drivetrainFrontLeftAngleMotor.getDeviceID() == 4) {

      kPSpecial = .4;
      //kModuleMaxAngularVelocity = Math.PI;
     // kModuleMaxAngularAcceleration = Math.PI/2;
    } else {
      kPSpecial = 1.2;
      kD = 0.02;
    }

    m_turningPIDController = new ProfiledPIDController(kPSpecial, 0.0, 0.02,
        new TrapezoidProfile.Constraints(kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration));

    m_driveMotor = drivetrainFrontLeftDriveMotor;
    m_turningMotor = drivetrainFrontLeftAngleMotor;
    m_turningEncoder = drivetrainFrontLeftAngleEncoder;

    angleOffset = angleChange;

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // loolution.

    // Set the distance (in this case, angle) per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * wpi::math::pi)
    // divided by the encoder resolution.
    // m_turningEncoder.setDistancePerPulse(2 * Math.PI / kEncoderResolution);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);

  }

  public double getTalonFXRate() {
    double ticksPerSec = m_driveMotor.getSelectedSensorVelocity(0) * 10;
    double revsPerSec = ticksPerSec / (kEncoderResolution * 8.307692307692308);
    double metersPerSec = revsPerSec * 2 * Math.PI * kWheelRadius;
    return -metersPerSec;
  }

  public double getTalonFXPos() {
    double ticks = m_driveMotor.getSelectedSensorPosition(0) * 10;
    double revs = ticks / (kEncoderResolution * 8.307692307692308);
    double meters = revs * 2 * Math.PI * kWheelRadius;
    return -meters;
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(getTalonFXRate(), new Rotation2d(getAngle()));
  }

  public void readAngle() {
    double angle = ((1 - (m_turningEncoder.getVoltage() / RobotController.getVoltage5V())) * 2.0 * Math.PI + angleOffset
        + 2 * Math.PI);
    angle %= 2 * Math.PI;
    angle -= Math.PI;
    m_steeringAngle = angle;
  }

  public double getAngle() {
    //readAngle();
    return m_steeringAngle;
  }

  /*
   * public double convertAngle(SwerveModuleState state) { double position =
   * readAngle(); double setpoint = state.angle.getRadians(); double newSetpoint =
   * state.angle.getRadians(); double finalSetpoint;
   * 
   * if(setpoint > 0) { newSetpoint -= Math.PI; } else { newSetpoint += Math.PI; }
   * 
   * double oldError = Math.abs(setpoint - position); double newError =
   * Math.abs(newSetpoint - position);
   * 
   * if(oldError > newError) { finalSetpoint = newSetpoint; } else { finalSetpoint
   * = setpoint; }
   * 
   * return 0; //WRONG!!!!!!! }
   */

  public double bound(double setpoint) {
    double dTheta = (setpoint + Math.PI) - (getAngle() + Math.PI);
    double trueDTheta = Math.IEEEremainder(dTheta, Math.PI);

    if (Math.abs(
        Math.IEEEremainder(getAngle() + trueDTheta, 2 * Math.PI) - Math.IEEEremainder(setpoint, 2 * Math.PI)) < .01) {
      m_driveScalar = 1;
    } else {
      m_driveScalar = -1;
    }

    if (Math.abs(trueDTheta) < Math.PI/2) {
      return getAngle() + trueDTheta;
    } else {
      return getAngle() + (trueDTheta - Math.PI) ;
    }
    
//New Bound
    // final double wrap = 2 * Math.PI; // in encoder counts
    // final double current = getAngle() ;//+ Math.PI ;
    // double newPosition;


    // if(false){
    //     newPosition = minChange(setpoint, current, wrap / 2.0) + current;
    //     if(Math.abs(minChange(newPosition, setpoint, wrap)) < .001){ // check if equal
    //         m_driveScalar = 1;
    //     } else {
    //         m_driveScalar = -1;
    //     }
    // }
    //   else {
    //     m_driveScalar = 1;
    //     newPosition = minChange(setpoint, current, wrap) + current;
    // }

    // return newPosition;// - Math.PI;
    
    /*
     * if (angle > (Math.PI / 2)) { m_driveScalar = -m_driveScalar; return angle -
     * Math.PI; } else if (angle < (-Math.PI / 2)) { m_driveScalar = -m_driveScalar;
     * return angle + Math.PI; } else { return angle; }
     */

  }
  public double minChange(double a, double b, double wrap) {
    return Math.IEEEremainder(a - b, wrap);
}
  /*
   * public double bound (double angle, double setpoint) { change1 =
   * setpoint-angle change2 = setpoint + Math.PI - angle bestChange = Math. }
   */
  /**
   * Sets the desired state for the module.
   *
   * @param state Desired state with speed and angle.
   */

  public double getDriveOutput()
  {
    return driveOutput;
  }
  public void setDesiredState(SwerveModuleState state) {
   readAngle();
    m_driveScalar = 1;
    double setpoint = bound(state.angle.getRadians());
    double currentAngle = getAngle(); 
    var turnOutput = m_turningPIDController.calculate(currentAngle, setpoint);
    
    driveOutput = m_drivePIDController.calculate(getTalonFXRate(), state.speedMetersPerSecond) *m_driveScalar;
  

    m_turningMotor.set(-turnOutput);
    m_driveMotor.set(driveOutput);
      
    System.out.println("measurement:" + currentAngle + ", setpoint: " +setpoint + ", = (turn output) " + turnOutput);
    SmartDashboard.putNumber("Setpoint bound angle: " + m_driveMotor.getDeviceID(), Math.toDegrees(setpoint));
    SmartDashboard.putNumber("setpoint unbound angle: " + m_driveMotor.getDeviceID(), state.angle.getDegrees());
    SmartDashboard.putNumber("Current unbound angle: " + m_driveMotor.getDeviceID(), Math.toDegrees(getAngle()));
    SmartDashboard.putNumber("turn output: " + m_driveMotor.getDeviceID(), turnOutput);
  

  }

  public void toSmartDashboard() {
    SmartDashboard.putNumber("voltage" + m_driveMotor.getDeviceID(), m_turningEncoder.getVoltage());

  }
}