package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Limelight.LedMode;
import frc.robot.RobotMap.SensorMap;

public class Drivetrain {

  private static Drivetrain instance;
  private PIDController robotAnglePID = new PIDController(0.5, 0, 0);
  static {
    instance = new Drivetrain();
  }

  public static Drivetrain getInstance() {
    return instance;
  }

  private Drivetrain() {
    SensorMap.GYRO.reset();
    robotAnglePID.enableContinuousInput(-Math.PI, Math.PI);
  }

 
  
  /**
   * 
   * Returns the angle of the robot as a Rotation2d.
   *
   * @return The angle of the robot.
   */
  public Rotation2d getAngle() {
    // Negating the angle because WPILib gyros are CW positive. CHECK WHEN FRAMES
    // CHANGE
    return Rotation2d.fromDegrees(-SensorMap.GYRO.getYaw());
  }

  /**
   * Method to drive the robot using joystick info.
   *
   * @param xSpeed        Speed of the robot in the x direction (forward).
   * @param ySpeed        Speed of the robot in the y direction (sideways).
   * @param rot           Angular rate of the robot.
   * @param fieldRelative Whether the provided x and y speeds are relative to the
   *                      field.
   */
  @SuppressWarnings("ParameterName")
  public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
    SwerveModuleState[] swerveModuleStates = RobotMap.DrivetrainMap.KINEMATICS
        .toSwerveModuleStates(fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getAngle())
            : new ChassisSpeeds(xSpeed, ySpeed, rot));
    /*
     * ***************************************************** 1 0 Xfl 0 1 Yfl 1 0 Xfr
     * XSpeed 0 1 Yfr MULTIPLIED BY SPEED VECTOR: YSpeed 1 0 Xbl ROTATION Responds
     * violently when you have ultrawide robots 0 1 Ybl 1 0 Xbr 0 1 YBr
     */
    setModuleStates(swerveModuleStates);
    
  }

  public void setModuleStates(SwerveModuleState[] swerveModuleStates) {
    SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, RobotMap.DriveMap.MAX_SPEED);
    RobotMap.DrivetrainMap.FRONT_LEFT.setDesiredState(swerveModuleStates[0]);
    RobotMap.DrivetrainMap.FRONT_RIGHT.setDesiredState(swerveModuleStates[1]);
    RobotMap.DrivetrainMap.BACK_LEFT.setDesiredState(swerveModuleStates[2]);
    RobotMap.DrivetrainMap.BACK_RIGHT.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Stops the robot.
   * <p>
   * Alias for {@link #drive(double, double, double, boolean) drive(0, 0, 0,
   * false)}.
   */
  public void stop() {
    drive(0, 0, 0, false);
  }
 
  /**
   * Updates the field relative position of the robot.
   */

  public void updateOdometry() {
    RobotMap.DrivetrainMap.ODOMETRY.update(getAngle(), RobotMap.DrivetrainMap.FRONT_LEFT.getState(),
        RobotMap.DrivetrainMap.FRONT_RIGHT.getState(), RobotMap.DrivetrainMap.BACK_LEFT.getState(),
        RobotMap.DrivetrainMap.BACK_RIGHT.getState());
    // System.out.println(RobotMap.DrivetrainMap.FRONT_LEFT.getState());
    // System.out.println(RobotMap.DrivetrainMap.FRONT_RIGHT.getState());
    // System.out.println(RobotMap.DrivetrainMap.BACK_LEFT.getState());
    // System.out.println(RobotMap.DrivetrainMap.BACK_RIGHT.getState());
  }

  public double getRightSidePos() {
    return RobotMap.DrivetrainMap.FRONT_RIGHT.getTalonFXPos();
  }

  public double getLeftSidePos() {
    return RobotMap.DrivetrainMap.FRONT_LEFT.getTalonFXPos();
  }

  public double getRightSideRate() {
    return RobotMap.DrivetrainMap.FRONT_RIGHT.getTalonFXRate();
  }

  public double getLeftSideRate() {
    return RobotMap.DrivetrainMap.FRONT_LEFT.getTalonFXRate();
  }

  public void postToSmartDashboard() {
    SmartDashboard.putNumber("front-right angle - (2,3)", RobotMap.DrivetrainMap.FRONT_RIGHT.getAngle());
    SmartDashboard.putNumber("front-left angle - (0,1)", RobotMap.DrivetrainMap.FRONT_LEFT.getAngle());
    SmartDashboard.putNumber("back-right angle - (6,7)", RobotMap.DrivetrainMap.BACK_RIGHT.getAngle());
    SmartDashboard.putNumber("back-left angle - (4,5)", RobotMap.DrivetrainMap.BACK_LEFT.getAngle());
    SmartDashboard.putNumber("Navx value", getAngle().getDegrees());
    SmartDashboard.putNumber("Joysticks y", RobotMap.CONTROLLER.getY(Hand.kLeft));
    RobotMap.DrivetrainMap.BACK_RIGHT.toSmartDashboard();
    RobotMap.DrivetrainMap.BACK_LEFT.toSmartDashboard();

    SmartDashboard.putNumber("meters dist X", RobotMap.DrivetrainMap.ODOMETRY.getPoseMeters().getTranslation().getX());
    SmartDashboard.putNumber("meters dist Y", RobotMap.DrivetrainMap.ODOMETRY.getPoseMeters().getTranslation().getY());
    SmartDashboard.putNumber("FR encoder rate", RobotMap.DrivetrainMap.FRONT_RIGHT.getTalonFXRate());
    SmartDashboard.putNumber("FL encoder rate", RobotMap.DrivetrainMap.FRONT_LEFT.getTalonFXRate());

    SmartDashboard.putNumber("FR drive output", RobotMap.DrivetrainMap.FRONT_RIGHT.getDriveOutput());
    SmartDashboard.putNumber("FL drive output", RobotMap.DrivetrainMap.FRONT_LEFT.getDriveOutput());
    SmartDashboard.putNumber("BL drive output", RobotMap.DrivetrainMap.BACK_LEFT.getDriveOutput());

    SmartDashboard.putNumber("FR state", RobotMap.DrivetrainMap.FRONT_RIGHT.getWheelState());
    SmartDashboard.putNumber("FL state", RobotMap.DrivetrainMap.FRONT_LEFT.getWheelState());

  }


  public void updateModuleAngles() {
    RobotMap.DrivetrainMap.BACK_LEFT.readAngle();
    RobotMap.DrivetrainMap.BACK_RIGHT.readAngle();
    RobotMap.DrivetrainMap.FRONT_LEFT.readAngle();
    RobotMap.DrivetrainMap.FRONT_RIGHT.readAngle();
  }

  public void teleopDrive()
  {
    double xSpeed = -1 * Math.pow(Math.abs(RobotMap.CONTROLLER.getY(Hand.kLeft)), 2)
        * Math.signum(RobotMap.CONTROLLER.getY(Hand.kLeft));
    if (Math.abs(xSpeed) < 0.2) {
      xSpeed = 0;
    } else { //this is the right way to type else
      xSpeed = (Math.abs(xSpeed)-.2) * (1/.8) * Math.signum(RobotMap.CONTROLLER.getY(Hand.kLeft));
    }
    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    double ySpeed = -1 * Math.pow(Math.abs(RobotMap.CONTROLLER.getX(Hand.kLeft)), 2)
        * Math.signum(RobotMap.CONTROLLER.getX(Hand.kLeft));
    if (Math.abs(ySpeed) < 0.2 ){
      ySpeed = 0;
    } else {
      ySpeed = (Math.abs(ySpeed)-.2) * (1/.8) * Math.signum(RobotMap.CONTROLLER.getX(Hand.kLeft));
    }
   
    double rot = -1 * Math.pow(Math.abs(RobotMap.CONTROLLER.getX(Hand.kRight)), 2)
    * Math.signum(RobotMap.CONTROLLER.getX(Hand.kRight));
    if (Math.abs(rot) < 0.2) {
      rot = 0;
    } else {
      rot = (Math.abs(rot)-.2) * (1/.8) * Math.signum(RobotMap.CONTROLLER.getX(Hand.kRight));
    }
  
    if(RobotMap.CONTROLLER.getBButton())
    {
      Limelight.setLED(LedMode.Current);
      rot = TargetAlignDrive.getInstance().align();
      //System.out.println("here3");
     
    }
    else if(rot == 0 && RobotMap.StateConstants.ALLOW_AUTOMATED_CONTROL)
    {
      robotAnglePID.setSetpoint(RobotMap.StateChooser.DRIVE_ANGLE);
      Limelight.setLED(LedMode.Current);
      if(RobotMap.StateChooser.DRIVE_ANGLE == 999)
      {
        rot = TargetAlignDrive.getInstance().align();
        //System.out.println("here1: " + rot);
      }
      else if (Math.abs(RobotMap.StateChooser.DRIVE_ANGLE - getAngle().getRadians()) < 3 * (Math.PI/180))
      {
        rot = TargetAlignDrive.getInstance().align();
        //System.out.println("here2: " + robotAnglePID.getPositionError());
      }
      else
      {
      rot = -robotAnglePID.calculate(getAngle().getRadians());
      //System.out.println("here");
      }
    }
    else
    
    {
      Limelight.setLED(LedMode.ForceOff);
    }
    // System.out.println("rot: " + robotMap.controller.getX(Hand.kRight));
    // System.out.println("rot-c: " + rot);
    // System.out.println(xSpeed + "," + ySpeed);
    
    drive(xSpeed * RobotMap.DriveMap.MAX_SPEED, ySpeed * RobotMap.DriveMap.MAX_SPEED, rot * RobotMap.DriveMap.MAX_ANGULAR_SPEED, RobotMap.StateChooser.FIELD_RELATIVE);

  }

/*
 public void turnToLongshot(){
    if(RobotMap.CONTROLLER.getYButtonPressed()){
      driveAtAngle(-.209, true);
    }
    else{
      //keepAngle();
    }
    
  }
*/
}
