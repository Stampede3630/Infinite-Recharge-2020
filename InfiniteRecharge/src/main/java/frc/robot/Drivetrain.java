package frc.robot;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.kauailabs.navx.frc.AHRS;
public class Drivetrain {

  //******************THESE locations must be in Meters ..... SwerveDriveKinematics computes in Meters****************** */
  //Ensure GYRo reading is not crazy (we may need to do a full long reset)
  // translation is (x,y) where x is forward and y is side-side
    private final Translation2d m_frontLeftLocation = new Translation2d(0.2794, 0.3302);
    private final Translation2d m_frontRightLocation = new Translation2d(0.2794, -0.3302);
    private final Translation2d m_backLeftLocation = new Translation2d(-0.2794, 0.3302);
    private final Translation2d m_backRightLocation = new Translation2d(-0.2794, -0.3302);

    private final SwerveModule m_frontLeft = new SwerveModule(RobotMap.DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR, RobotMap.DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR, RobotMap.DRIVETRAIN_FRONT_LEFT_ANGLE_ENCODER, RobotMap.FRONT_LEFT_ANGLE_OFFSET);
    private final SwerveModule m_frontRight = new SwerveModule(RobotMap.DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR, RobotMap.DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR, RobotMap.DRIVETRAIN_FRONT_RIGHT_ANGLE_ENCODER, RobotMap.FRONT_RIGHT_ANGLE_OFFSET);
    private final SwerveModule m_backLeft = new SwerveModule(RobotMap.DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR, RobotMap.DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR, RobotMap.DRIVETRAIN_BACK_LEFT_ANGLE_ENCODER, RobotMap.BACK_LEFT_ANGLE_OFFSET);
    private final SwerveModule m_backRight = new SwerveModule(RobotMap.DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR, RobotMap.DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR, RobotMap.DRIVETRAIN_BACK_RIGHT_ANGLE_ENCODER, RobotMap.BACK_RIGHT_ANGLE_OFFSET);
  
    private final AHRS m_gyro = new AHRS(SPI.Port.kMXP);
  
    private final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(
        m_frontLeftLocation, m_frontRightLocation, m_backLeftLocation, m_backRightLocation
    );

  
    //private final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics, getAngle());
  
    public Drivetrain() {
      m_gyro.reset();
    }
  
    /**
     * Returns the angle of the robot as a Rotation2d.
     *
     * @return The angle of the robot.
     */
    public Rotation2d getAngle() {
      // Negating the angle because WPILib gyros are CW positive. CHECK WHEN FRAMES CHANGE
      return Rotation2d.fromDegrees(-m_gyro.getAngle());
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


      SwerveDriveKinematics.normalizeWheelSpeeds(swerveModuleStates, Robot.kMaxSpeed);
     m_frontLeft.setDesiredState(swerveModuleStates[0]);
     m_frontRight.setDesiredState(swerveModuleStates[1]);
      m_backLeft.setDesiredState(swerveModuleStates[2]);
     m_backRight.setDesiredState(swerveModuleStates[3]);
    }
  
    /**
     * Updates the field relative position of the robot.
     */
    /*
    public void updateOdometry() {
      m_odometry.update(
          getAngle(),
          m_frontLeft.getState(),
          m_frontRight.getState(),
          m_backLeft.getState(),
          m_backRight.getState()
      );
    }
    */

    public void postToSmartDashboard()
    {
      SmartDashboard.putNumber("front-right angle - (2,3)", m_frontRight.readAngle());
      SmartDashboard.putNumber("front-left angle - (0,1)",m_frontLeft.readAngle());
     SmartDashboard.putNumber("back-right angle - (6,7)", m_backRight.readAngle());
      SmartDashboard.putNumber("back-left angle - (4,5)", m_backLeft.readAngle());
      SmartDashboard.putNumber("Navx value",getAngle().getDegrees());
    }
  }
