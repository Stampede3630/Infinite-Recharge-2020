package frc.robot;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SpeedController;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


public class SwerveModule {
  //private static final double kWheelRadius = 0.0508;
  //private static final int kEncoderResolution = 4096;

  private static final double kModuleMaxAngularVelocity = Math.PI * 2 * 2.6;
  private static final double kModuleMaxAngularAcceleration = 4 * Math.PI; // radians per second squared

  private final WPI_TalonFX m_driveMotor;
  private final WPI_TalonSRX m_turningMotor;

  //private final Encoder m_driveEncoder
  private final AnalogInput m_turningEncoder;

  private final PIDController m_drivePIDController = new PIDController(0.05, 0, 0);

  //private final PIDController m_turningPIDController = new PIDController((1.5/Math.PI),0.0, (.05/Math.PI));//.5, 0, 0
  private final PIDController m_turningPIDController = new PIDController((.5/Math.PI), 0.0, 0.0);//.5, 0, 0

     // new TrapezoidProfile.Constraints(kModuleMaxAngularVelocity, kModuleMaxAngularAcceleration)
  //front left -(0,1) - PID: 0.8, 0.05
  //(4,5) -PID: 0.8, 0.05
  private final double angleOffset;

  /**
   * Constructs a SwerveModule.
   *
   * @param driveMotorChannel   ID for the drive motor.
   * @param turningMotorChannel ID for the turning motor.
   */
  public SwerveModule(int driveMotorChannel, int turningMotorChannel, int angleEncoder, double angleChange) {
    m_driveMotor = new WPI_TalonFX(driveMotorChannel);
    m_turningMotor = new WPI_TalonSRX(turningMotorChannel);
    //if(turningMotorChannel != 4){}
    m_turningMotor.setInverted(true);
    
    
   
    m_turningEncoder = new AnalogInput(angleEncoder);
    angleOffset = angleChange;

    // Set the distance per pulse for the drive encoder. We can simply use the
    // distance traveled for one rotation of the wheel divided by the encoder
    // resolution.
    //m_driveEncoder.setDistancePerPulse(2 * Math.PI * kWheelRadius / kEncoderResolution);

    // Set the distance (in this case, angle) per pulse for the turning encoder.
    // This is the the angle through an entire rotation (2 * wpi::math::pi)
    // divided by the encoder resolution.
    //m_turningEncoder.setDistancePerPulse(2 * Math.PI / kEncoderResolution);

    // Limit the PID Controller's input range between -pi and pi and set the input
    // to be continuous.
    m_turningPIDController.enableContinuousInput(-Math.PI, Math.PI);
  }

  /**
   * Returns the current state of the module.
   *
   * @return The current state of the module.
   */
  public SwerveModuleState getState() {
    return new SwerveModuleState(0, new Rotation2d(readAngle()));
  }

  public double readAngle()
  {
    double angle = ((1-(m_turningEncoder.getVoltage()/RobotController.getVoltage5V())) * 2.0 * Math.PI + angleOffset + 2 *Math.PI);
    angle %= 2 * Math.PI;
    angle -= Math.PI;
    return angle;
  }
  public void toSmartDashboard(){
    SmartDashboard.putNumber("voltage" + m_driveMotor.getDeviceID(), m_turningEncoder.getVoltage());

  }
/*
  public double convertAngle(SwerveModuleState state)
  {
    double position = readAngle();
    double setpoint = state.angle.getRadians();
    double newSetpoint = state.angle.getRadians();
    double finalSetpoint;
   
    if(setpoint > 0)
    {
      newSetpoint -= Math.PI;
    }
    else
    {
      newSetpoint += Math.PI;
    }
    
    double oldError = Math.abs(setpoint - position);
    double newError = Math.abs(newSetpoint - position);
    
    if(oldError > newError)
    {
      finalSetpoint = newSetpoint;
    }
    else
    {
     finalSetpoint = setpoint;
    }

      return 0; //WRONG!!!!!!!
    

  }
*/

  public double bound (double angle)
  {
    if(angle > (Math.PI/2))
    {
      return angle - Math.PI;
    }
    else if (angle < (-Math.PI/2))
    {
      return angle + Math.PI;
    }
    else
    {
      return angle;
    }
  }
  /**
   * Sets the desired state for the module.
   *
   * @param state Desired state with speed and angle.
   */
  public void setDesiredState(SwerveModuleState state) {
    // Calculate the drive output from the drive PID controller.
    //var driveOutput = m_drivePIDController.calculate(
       // 0, /*INSERT DRIVE ENCODER VALUE HERE*/ 0);

    // Calculate the turning motor output from the turning PID controller.
    double setpoint = bound(state.angle.getRadians());
    double currentAngle = bound(readAngle());
    var turnOutput = m_turningPIDController.calculate(
        currentAngle, setpoint);

    System.out.println(m_driveMotor.getDeviceID() + "," + "Current angle" + currentAngle  + "setpoint"  +setpoint + ",     turn output:  " + turnOutput);

    /*if (m_turningMotor.getDeviceID() == 4){

      turnOutput = m_turningPIDController.calculate(currentAngle, setpoint/4);
    }*/
    double driveOutput = state.speedMetersPerSecond / Robot.kMaxSpeed;
   /*
    if(driveOutput == 0)
    {
      turnOutput = 0;
    }
    */
    // Calculate the turning motor output from the turning PID controller.
    //m_driveMotor.set(driveOutput);
    m_turningMotor.set(-turnOutput);
    m_driveMotor.set(driveOutput);
    //System.out.println(m_driveMotor.getDeviceID() + "," + turnOutput + "," + driveOutput );
    //SUPER FANCY MATH TO NORMALIZE WHEEL SPEED
    
    if(state.angle.getRadians() != setpoint)
    {
        driveOutput = driveOutput * -1;
        System.out.println("State of " + m_driveMotor.getDeviceID() + " Has Been Bounded !!!!!!!!!!!!!!!!!!!!");
    }
    if(readAngle() != currentAngle)
    {
      driveOutput = driveOutput *-1;
      System.out.println("Measurement of " + m_driveMotor.getDeviceID() + " Has Been Bounded !!!!!!!!!!!!!!!!!!!!");
    }
    /*
    if((Math.abs(setpoint - readAngle())) < 2)
    {
      m_driveMotor.set(driveOutput);
    }
    */

    
    //System.out.println(state.speedMetersPerSecond);
    
    //System.out.println("Current State" +  readAngle() + " requested state," + state.angle.getRadians() + "turn output: " + turnOutput);
  }
}