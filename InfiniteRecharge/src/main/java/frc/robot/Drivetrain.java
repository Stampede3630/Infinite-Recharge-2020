package frc.robot;

import com.kauailabs.navx.frc.AHRS;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap.SensorMap;

public class Drivetrain {

  private static Drivetrain instance;
  private PIDController robotAnglePID = new PIDController(0.008, 0, 0);
  static {
    instance = new Drivetrain();
  }

  public static Drivetrain getInstance() {
    return instance;
  }

  private Drivetrain() {
    SensorMap.GYRO.reset();
    robotAnglePID.enableContinuousInput(-180, 180);
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

  }

  /**
   * Updates the field relative position of the robot.
   */

  public void driveWithJoystick(boolean fieldRelative) {

    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
    double xSpeed = 1 * Math.pow(Math.abs(RobotMap.CONTROLLER.getY(Hand.kLeft)), 2)
        * Math.signum(RobotMap.CONTROLLER.getY(Hand.kLeft)) * RobotMap.DriveMap.MAX_SPEED;
    if (Math.abs(xSpeed) < (0.2 * RobotMap.DriveMap.MAX_SPEED)) {
      xSpeed = 0;
    }
    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    double ySpeed = 1 * Math.pow(Math.abs(RobotMap.CONTROLLER.getX(Hand.kLeft)), 2)
        * Math.signum(RobotMap.CONTROLLER.getX(Hand.kLeft)) * RobotMap.DriveMap.MAX_SPEED;
    if (Math.abs(ySpeed) < (0.2 * RobotMap.DriveMap.MAX_SPEED)) {
      ySpeed = 0;
    }
    // Get the rate of angular rotatpion. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    double rot = 1 * Math.pow(Math.abs(RobotMap.CONTROLLER.getX(Hand.kRight)), 2)
        * Math.signum(RobotMap.CONTROLLER.getX(Hand.kRight)) * RobotMap.DriveMap.MAX_ANGULAR_SPEED;

    if (Math.abs(rot) < (0.2 * RobotMap.DriveMap.MAX_ANGULAR_SPEED)) {
      rot = 0;
    }
    // System.out.println("rot: " + robotMap.controller.getX(Hand.kRight));
    // System.out.println("rot-c: " + rot);
    // System.out.println(xSpeed + "," + ySpeed);
    drive(xSpeed, ySpeed, rot, fieldRelative);

  }
  
  public void updateModuleAngles() {
    RobotMap.DrivetrainMap.BACK_LEFT.readAngle();
    RobotMap.DrivetrainMap.BACK_RIGHT.readAngle();
    RobotMap.DrivetrainMap.FRONT_LEFT.readAngle();
    RobotMap.DrivetrainMap.FRONT_RIGHT.readAngle();
  }

  public void driveAtAngle(double angle, boolean fieldRelative)
  {
    var xSpeed = -Math.pow(Math.abs(RobotMap.CONTROLLER.getY(Hand.kLeft)), 2) * Math.signum(RobotMap.CONTROLLER.getY(Hand.kLeft)) * RobotMap.DriveMap.MAX_SPEED;
    if (Math.abs(xSpeed) < (0.2 * RobotMap.DriveMap.MAX_SPEED)) {
      xSpeed = 0;
    }

    // Get the y speed or sideways/strafe speed. We are inverting this because
    // we want a positive value when we pull to the left. Xbox controllers
    // return positive values when you pull to the right by default.
    var ySpeed = -Math.pow(Math.abs(RobotMap.CONTROLLER.getX(Hand.kLeft)), 2) * Math.signum(RobotMap.CONTROLLER.getX(Hand.kLeft)) * RobotMap.DriveMap.MAX_SPEED;
    if (Math.abs(ySpeed) < (0.2 * RobotMap.DriveMap.MAX_SPEED)) {
      ySpeed = 0;
    }
    // Get the rate of angular rotatpion. We are inverting this because we want a
    // positive value when we pull to the left (remember, CCW is positive in
    // mathematics). Xbox controllers return positive values when you pull to
    // the right by default.
    double turnSpeed = -robotAnglePID.calculate(getAngle().getDegrees(), angle) *RobotMap.DriveMap.MAX_ANGULAR_SPEED;
    SmartDashboard.putNumber("turnSpeed", turnSpeed);
    SmartDashboard.putNumber("current angle", getAngle().getDegrees());
    drive(xSpeed, ySpeed, turnSpeed, fieldRelative);
  }


 public void turnToLongshot(){
    if(RobotMap.CONTROLLER.getYButtonPressed()){
      driveAtAngle(-.209, true);
    }
    else{
      //keepAngle();
    }
    
  }

}