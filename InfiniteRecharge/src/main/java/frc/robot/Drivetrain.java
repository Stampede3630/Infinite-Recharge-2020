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
    SwerveModuleState[] swerveModuleStates = RobotMap.DriveMap.KINEMATICS
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
    RobotMap.DriveMap.FRONT_LEFT.setDesiredState(swerveModuleStates[0]);
    RobotMap.DriveMap.FRONT_RIGHT.setDesiredState(swerveModuleStates[1]);
    RobotMap.DriveMap.BACK_LEFT.setDesiredState(swerveModuleStates[2]);
    RobotMap.DriveMap.BACK_RIGHT.setDesiredState(swerveModuleStates[3]);
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
    RobotMap.DriveMap.ODOMETRY.update(getAngle(), RobotMap.DriveMap.FRONT_LEFT.getState(),
        RobotMap.DriveMap.FRONT_RIGHT.getState(), RobotMap.DriveMap.BACK_LEFT.getState(),
        RobotMap.DriveMap.BACK_RIGHT.getState());
  }

  public double getRightSidePos() {
    return RobotMap.DriveMap.FRONT_RIGHT.getTalonFXPos();
  }

  public double getLeftSidePos() {
    return RobotMap.DriveMap.FRONT_LEFT.getTalonFXPos();
  }

  public double getRightSideRate() {
    return RobotMap.DriveMap.FRONT_RIGHT.getTalonFXRate();
  }

  public double getLeftSideRate() {
    return RobotMap.DriveMap.FRONT_LEFT.getTalonFXRate();
  }

  public void postToSmartDashboard() {
    SmartDashboard.putNumber("front-right angle - (2,3)", RobotMap.DriveMap.FRONT_RIGHT.getAngle());
    SmartDashboard.putNumber("front-left angle - (0,1)", RobotMap.DriveMap.FRONT_LEFT.getAngle());
    SmartDashboard.putNumber("back-right angle - (6,7)", RobotMap.DriveMap.BACK_RIGHT.getAngle());
    SmartDashboard.putNumber("back-left angle - (4,5)", RobotMap.DriveMap.BACK_LEFT.getAngle());
    SmartDashboard.putNumber("Navx value", getAngle().getDegrees());
    SmartDashboard.putNumber("Joysticks y", RobotMap.CONTROLLER.getY(Hand.kLeft));
    RobotMap.DriveMap.BACK_RIGHT.toSmartDashboard();
    RobotMap.DriveMap.BACK_LEFT.toSmartDashboard();

    SmartDashboard.putNumber("meters dist X", RobotMap.DriveMap.ODOMETRY.getPoseMeters().getTranslation().getX());
    SmartDashboard.putNumber("meters dist Y", RobotMap.DriveMap.ODOMETRY.getPoseMeters().getTranslation().getY());
    SmartDashboard.putNumber("FR encoder rate", RobotMap.DriveMap.FRONT_RIGHT.getTalonFXRate());
    SmartDashboard.putNumber("FL encoder rate", RobotMap.DriveMap.FRONT_LEFT.getTalonFXRate());

    SmartDashboard.putNumber("FR drive output", RobotMap.DriveMap.FRONT_RIGHT.getDriveOutput());
    SmartDashboard.putNumber("FL drive output", RobotMap.DriveMap.FRONT_LEFT.getDriveOutput());
    SmartDashboard.putNumber("BL drive output", RobotMap.DriveMap.BACK_LEFT.getDriveOutput());

    SmartDashboard.putNumber("FR state", RobotMap.DriveMap.FRONT_RIGHT.getWheelState());
    SmartDashboard.putNumber("FL state", RobotMap.DriveMap.FRONT_LEFT.getWheelState());

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
    drive(xSpeed * RobotMap.DriveMap.MAX_SPEED, ySpeed * RobotMap.DriveMap.MAX_SPEED, rot * RobotMap.DriveMap.MAX_ANGULAR_SPEED, RobotMap.StateChooser.FIELD_RELATIVE);

  }

}
