/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.kauailabs.navx.frc.AHRS;
public class Drivetrain {

  //******************THESE locations must be in Meters ..... SwerveDriveKinematics computes in Meters****************** */
  //Ensure GYRo reading is not crazy (we may need to do a full long reset)
  // translation is (x,y) where x is forward and y is side-side
    private final Translation2d m_frontLeftLocation = new Translation2d(0.3556, 0.3556);
    private final Translation2d m_frontRightLocation = new Translation2d(0.3556, -0.3556);
    private final Translation2d m_backLeftLocation = new Translation2d(-0.3556, 0.3556);
    private final Translation2d m_backRightLocation = new Translation2d(-0.3556, -0.3556);
    private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

    public final SwerveModule m_frontLeft = new SwerveModule(RobotMap.DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR, RobotMap.DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR, RobotMap.DRIVETRAIN_FRONT_LEFT_ANGLE_ENCODER, RobotMap.FRONT_LEFT_ANGLE_OFFSET, m_gyro);
    public final SwerveModule m_frontRight = new SwerveModule(RobotMap.DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR, RobotMap.DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR, RobotMap.DRIVETRAIN_FRONT_RIGHT_ANGLE_ENCODER, RobotMap.FRONT_RIGHT_ANGLE_OFFSET, m_gyro);
    private final SwerveModule m_backLeft = new SwerveModule(RobotMap.DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR, RobotMap.DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR, RobotMap.DRIVETRAIN_BACK_LEFT_ANGLE_ENCODER, RobotMap.BACK_LEFT_ANGLE_OFFSET, m_gyro);
    private final SwerveModule m_backRight = new SwerveModule(RobotMap.DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR, RobotMap.DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR, RobotMap.DRIVETRAIN_BACK_RIGHT_ANGLE_ENCODER, RobotMap.BACK_RIGHT_ANGLE_OFFSET, m_gyro);

    public static final double kMaxSpeed = 4; // 3 meters per second
    public static final double kMaxAngularSpeed = Math.PI; // 1/2 rotation per second
  
    
  
    SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
        m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
    );

    final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, getAngle());

    static Drivetrain drivetrain;

    public Drivetrain() {
      m_gyro.reset();
    }

    public static Drivetrain getInstance(){
        if(drivetrain == null){
            drivetrain = new Drivetrain();
        }
        return drivetrain;
    }
  
    /**
     * 
     * Returns the angle of the robot as a Rotation2d.
     *
     * @return The angle of the robot.
     */
<<<<<<< Updated upstream
    public Rotation2d getAngle() {
      // Negating the angle because WPILib gyros are CW positive. CHECK WHEN FRAMES CHANGE
      return Rotation2d.fromDegrees(-m_gyro.getAngle() + 90);
    }
  
    /**
     * Method to drive the robot using joystick info.
     *
     * @param xSpeed        Speed of the robot in the x direction (forward).
     * @param ySpeed        Speed of the robot in the y direction (sideways).
     * @param rot           Angular rate of the robot.
     * @param fieldRelative Whether the provided x and y speeds are relative to the field.
     */
    @SuppressWarnings("ParameterName")
    public void drive(double xSpeed, double ySpeed, double rot, boolean fieldRelative) {
      var swerveModuleStates = m_kinematics.toSwerveModuleStates(
          fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(
              xSpeed, ySpeed, rot, getAngle())
              : new ChassisSpeeds(xSpeed, ySpeed, rot)
      );
    /* *****************************************************
    1 0 Xfl
    0 1 Yfl
    1 0 Xfr                                     XSpeed
    0 1 Yfr       MULTIPLIED BY SPEED VECTOR:   YSpeed
    1 0 Xbl                                     ROTATION Responds violently when you have ultrawide robots
    0 1 Ybl
    1 0 Xbr
    0 1 YBr
    ********************************************************* */


      SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, RobotMap.kMaxSpeed);
     m_frontLeft.setDesiredState(swerveModuleStates[0]);
     m_frontRight.setDesiredState(swerveModuleStates[1]);
      m_backLeft.setDesiredState(swerveModuleStates[2]);
     m_backRight.setDesiredState(swerveModuleStates[3]);
    }
    
    public void setModuleStates(SwerveModuleState[] swerveModuleStates)
    {
      SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, RobotMap.kMaxSpeed);
     m_frontLeft.setDesiredState(swerveModuleStates[0]);
     m_frontRight.setDesiredState(swerveModuleStates[1]);
      m_backLeft.setDesiredState(swerveModuleStates[2]);
     m_backRight.setDesiredState(swerveModuleStates[3]);
    }
    /**
     * Updates the field relative position of the robot.
     */
    
    public void updateOdometry() {
      m_odometry.update(
          getAngle(),
          m_frontLeft.getState(),
          m_frontRight.getState(),
          m_backLeft.getState(),
          m_backRight.getState()
      );
      //System.out.println(m_frontLeft.getState());
      //System.out.println(m_frontRight.getState());
      //System.out.println(m_backLeft.getState());
      //System.out.println(m_backRight.getState());
    }
    
    public double getRightSidePos()
    {
      return m_frontRight.getTalonFXPos();
    }
    public double getLeftSidePos()
    {
      return m_frontLeft.getTalonFXPos();
    }

    public double getRightSideRate()
    {
      return m_frontRight.getTalonFXRate();
=======
    setModuleStates(swerveModuleStates);
  }

  public static void setModuleStates(SwerveModuleState[] swerveModuleStates) {
    SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, RobotMap.DriveMap.MAX_SPEED);
    m_frontLeft.setDesiredState(swerveModuleStates[0]);
    m_frontRight.setDesiredState(swerveModuleStates[1]);
    m_backLeft.setDesiredState(swerveModuleStates[2]);
    m_backRight.setDesiredState(swerveModuleStates[3]);
  }

  /**
   * Stops the robot.
   * <p>
   * Alias for {@link #drive(double, double, double, boolean) drive(0, 0, 0, false)}.
   */
  public static void stop() {
    drive(0, 0, 0, false);
  }

  /**
   * Updates the field relative position of the robot.
   */

  public static void updateOdometry() {
    m_odometry.update(getAngle(), m_frontLeft.getState(), m_frontRight.getState(), m_backLeft.getState(),
        m_backRight.getState());
    // System.out.println(m_frontLeft.getState());
    // System.out.println(m_frontRight.getState());
    // System.out.println(m_backLeft.getState());
    // System.out.println(m_backRight.getState());
  }

  public static double getRightSidePos() {
    return m_frontRight.getTalonFXPos();
  }

  public static double getLeftSidePos() {
    return m_frontLeft.getTalonFXPos();
  }

  public static double getRightSideRate() {
    return m_frontRight.getTalonFXRate();
  }

  public static double getLeftSideRate() {
    return m_frontLeft.getTalonFXRate();
  }

  public static void postToSmartDashboard() {
    SmartDashboard.putNumber("front-right angle - (2,3)", m_frontRight.getAngle());
    SmartDashboard.putNumber("front-left angle - (0,1)", m_frontLeft.getAngle());
    SmartDashboard.putNumber("back-right angle - (6,7)", m_backRight.getAngle());
    SmartDashboard.putNumber("back-left angle - (4,5)", m_backLeft.getAngle());
    SmartDashboard.putNumber("Navx value", getAngle().getDegrees());
    SmartDashboard.putNumber("Joysticks y", RobotMap.CONTROLLER.getY(Hand.kLeft));
    m_backRight.toSmartDashboard();
    m_backLeft.toSmartDashboard();

    SmartDashboard.putNumber("meters dist X", m_odometry.getPoseMeters().getTranslation().getX());
    SmartDashboard.putNumber("meters dist Y", m_odometry.getPoseMeters().getTranslation().getY());
    SmartDashboard.putNumber("FR encoder rate", m_frontRight.getTalonFXRate());
    SmartDashboard.putNumber("FL encoder rate", m_frontLeft.getTalonFXRate());

    SmartDashboard.putNumber("FR drive output", m_frontRight.getDriveOutput());
    SmartDashboard.putNumber("FL drive output", m_frontLeft.getDriveOutput());
    SmartDashboard.putNumber("BL drive output", m_backLeft.getDriveOutput());


  }

  public static void driveWithJoystick(boolean fieldRelative) {

    // Get the x speed. We are inverting this because Xbox controllers return
    // negative values when we push forward.
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
    var rot = -Math.pow(Math.abs(RobotMap.CONTROLLER.getX(Hand.kRight)), 2) * Math.signum(RobotMap.CONTROLLER.getX(Hand.kRight)) * RobotMap.DriveMap.MAX_ANGULAR_SPEED;
    if (Math.abs(rot) < (0.2 * RobotMap.DriveMap.MAX_ANGULAR_SPEED)) {
      rot = 0;
    }
    // System.out.println("rot: " + robotMap.controller.getX(Hand.kRight));
    // System.out.println("rot-c: " + rot);
    //System.out.println(xSpeed + "," + ySpeed);
    drive(xSpeed, ySpeed, rot, fieldRelative);
  }

  public void updateModuleAngles()

  {
    m_backLeft.readAngle();
    m_backRight.readAngle();
    m_frontLeft.readAngle();
    m_frontRight.readAngle();
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
>>>>>>> Stashed changes
    }
    public double getLeftSideRate()
    {
      return m_frontLeft.getTalonFXRate();
    }



    public void postToSmartDashboard()
    {
      SmartDashboard.putNumber("front-right angle - (2,3)", m_frontRight.getAngle());
      SmartDashboard.putNumber("front-left angle - (0,1)",m_frontLeft.getAngle());
     SmartDashboard.putNumber("back-right angle - (6,7)", m_backRight.getAngle());
      SmartDashboard.putNumber("back-left angle - (4,5)", m_backLeft.getAngle());
      SmartDashboard.putNumber("Navx value",getAngle().getDegrees());

      m_backRight.toSmartDashboard();
      m_backLeft.toSmartDashboard();

      SmartDashboard.putNumber("meters dist X", m_odometry.getPoseMeters().getTranslation().getX());
      SmartDashboard.putNumber("meters dist Y", m_odometry.getPoseMeters().getTranslation().getY());

    }
    public void driveWithJoystick(boolean fieldRelative) {
        
        // Get the x speed. We are inverting this because Xbox controllers return
        // negative values when we push forward.
        var xSpeed = RobotMap.controller.getY(Hand.kLeft)* kMaxSpeed;
        if(Math.abs(xSpeed) < (0.2 * kMaxSpeed))
        {
          xSpeed = 0;
        }
        
        // Get the y speed or sideways/strafe speed. We are inverting this because
        // we want a positive value when we pull to the left. Xbox controllers
        // return positive values when you pull to the right by default.
        var ySpeed = RobotMap.controller.getX(Hand.kLeft) * kMaxSpeed;
        if(Math.abs(ySpeed) < (0.2 * kMaxSpeed))
        {
          ySpeed = 0;
        }
        // Get the rate of angular rotatpion. We are inverting this because we want a
        // positive value when we pull to the left (remember, CCW is positive in
        // mathematics). Xbox controllers return positive values when you pull to
        // the right by default.
        var rot = RobotMap.controller.getX(Hand.kRight) * kMaxAngularSpeed;
        if(Math.abs(rot) < (0.2 * kMaxAngularSpeed))
        {
          rot = 0;
        }
        //System.out.println("rot: " + robotMap.controller.getX(Hand.kRight));
        //System.out.println("rot-c: " + rot);
        System.out.println(xSpeed + "," + ySpeed);
        drive(xSpeed, ySpeed, rot, fieldRelative);
      }
  }