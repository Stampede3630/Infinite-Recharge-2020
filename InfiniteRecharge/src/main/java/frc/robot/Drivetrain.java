package frc.robot;

import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.kauailabs.navx.frc.AHRS;

public class Drivetrain {

	private static Drivetrain instance;

	static {
		instance = new Drivetrain();
	}

	public static Drivetrain getInstance() {
		return instance;
	}

	private Drivetrain() {
		m_gyro.reset();
	}

	private static final Translation2d m_frontLeftLocation = new Translation2d(0.3556, 0.3556);
	private static final Translation2d m_frontRightLocation = new Translation2d(0.3556, -0.3556);
	private static final Translation2d m_backLeftLocation = new Translation2d(-0.3556, 0.3556);
	private static final Translation2d m_backRightLocation = new Translation2d(-0.3556, -0.3556);
	private static final AHRS m_gyro = new AHRS(SPI.Port.kMXP);

	public static final SwerveModule m_frontLeft = new SwerveModule(RobotMap.DriveMap.FRONT_LEFT_DRIVE_MOTOR,
			RobotMap.DriveMap.FRONT_LEFT_ANGLE_MOTOR, RobotMap.DriveMap.FRONT_LEFT_ANGLE_ENCODER,
			RobotMap.DriveMap.FRONT_LEFT_ANGLE_OFFSET);
	public static final SwerveModule m_frontRight = new SwerveModule(RobotMap.DriveMap.FRONT_RIGHT_DRIVE_MOTOR,
			RobotMap.DriveMap.FRONT_RIGHT_ANGLE_MOTOR, RobotMap.DriveMap.FRONT_RIGHT_ANGLE_ENCODER,
			RobotMap.DriveMap.FRONT_RIGHT_ANGLE_OFFSET);
	private static final SwerveModule m_backLeft = new SwerveModule(RobotMap.DriveMap.BACK_LEFT_DRIVE_MOTOR,
			RobotMap.DriveMap.BACK_LEFT_ANGLE_MOTOR, RobotMap.DriveMap.BACK_LEFT_ANGLE_ENCODER,
			RobotMap.DriveMap.BACK_LEFT_ANGLE_OFFSET);
	private static final SwerveModule m_backRight = new SwerveModule(RobotMap.DriveMap.BACK_RIGHT_DRIVE_MOTOR,
			RobotMap.DriveMap.BACK_RIGHT_ANGLE_MOTOR, RobotMap.DriveMap.BACK_RIGHT_ANGLE_ENCODER,
			RobotMap.DriveMap.BACK_RIGHT_ANGLE_OFFSET);

	public static final SwerveDriveKinematics m_kinematics = new SwerveDriveKinematics(m_frontLeftLocation,
			m_frontRightLocation, m_backLeftLocation, m_backRightLocation);

	public static final SwerveDriveOdometry m_odometry = new SwerveDriveOdometry(m_kinematics,
			Rotation2d.fromDegrees(-m_gyro.getAngle()));
	private PIDController robotAnglePID = new PIDController(0.1, 0, 0);

	/**
	 * 
	 * Returns the angle of the robot as a Rotation2d.
	 *
	 * @return The angle of the robot.
	 */
	public Rotation2d getAngle() {
		// Negating the angle because WPILib gyros are CW positive. CHECK WHEN FRAMES
		// CHANGE
		return Rotation2d.fromDegrees(-m_gyro.getAngle());
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
		SwerveModuleState[] swerveModuleStates = m_kinematics.toSwerveModuleStates(
				fieldRelative ? ChassisSpeeds.fromFieldRelativeSpeeds(xSpeed, ySpeed, rot, getAngle())
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
		m_frontLeft.setDesiredState(swerveModuleStates[0]);
		m_frontRight.setDesiredState(swerveModuleStates[1]);
		m_backLeft.setDesiredState(swerveModuleStates[2]);
		m_backRight.setDesiredState(swerveModuleStates[3]);
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
		m_odometry.update(getAngle(), m_frontLeft.getState(), m_frontRight.getState(), m_backLeft.getState(),
				m_backRight.getState());
		// System.out.println(m_frontLeft.getState());
		// System.out.println(m_frontRight.getState());
		// System.out.println(m_backLeft.getState());
		// System.out.println(m_backRight.getState());
	}

	public double getRightSidePos() {
		return m_frontRight.getTalonFXPos();
	}

	public double getLeftSidePos() {
		return m_frontLeft.getTalonFXPos();
	}

	public double getRightSideRate() {
		return m_frontRight.getTalonFXRate();
	}

	public double getLeftSideRate() {
		return m_frontLeft.getTalonFXRate();
	}

	public void postToSmartDashboard() {
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

	/**
	 * Updates the field relative position of the robot.
	 */

	public void driveWithJoystick(boolean fieldRelative) {

		// Get the x speed. We are inverting this because Xbox controllers return
		// negative values when we push forward.
		double xSpeed = -Math.pow(Math.abs(RobotMap.CONTROLLER.getY(Hand.kLeft)), 2)
				* Math.signum(RobotMap.CONTROLLER.getY(Hand.kLeft)) * RobotMap.DriveMap.MAX_SPEED;
		if (Math.abs(xSpeed) < (0.2 * RobotMap.DriveMap.MAX_SPEED)) {
			xSpeed = 0;
		}

		// Get the y speed or sideways/strafe speed. We are inverting this because
		// we want a positive value when we pull to the left. Xbox controllers
		// return positive values when you pull to the right by default.
		double ySpeed = -Math.pow(Math.abs(RobotMap.CONTROLLER.getX(Hand.kLeft)), 2)
				* Math.signum(RobotMap.CONTROLLER.getX(Hand.kLeft)) * RobotMap.DriveMap.MAX_SPEED;
		if (Math.abs(ySpeed) < (0.2 * RobotMap.DriveMap.MAX_SPEED)) {
			ySpeed = 0;
		}
		// Get the rate of angular rotatpion. We are inverting this because we want a
		// positive value when we pull to the left (remember, CCW is positive in
		// mathematics). Xbox controllers return positive values when you pull to
		// the right by default.
		double rot = -Math.pow(Math.abs(RobotMap.CONTROLLER.getX(Hand.kRight)), 2)
				* Math.signum(RobotMap.CONTROLLER.getX(Hand.kRight)) * RobotMap.DriveMap.MAX_ANGULAR_SPEED;
		if (Math.abs(rot) < (0.2 * RobotMap.DriveMap.MAX_ANGULAR_SPEED)) {
			rot = 0;
		}
		// System.out.println("rot: " + robotMap.controller.getX(Hand.kRight));
		// System.out.println("rot-c: " + rot);
		// System.out.println(xSpeed + "," + ySpeed);
		drive(xSpeed, ySpeed, rot, fieldRelative);
	}

	public void driveAtAngle(double angle, boolean fieldRelative) {
		double xSpeed = -Math.pow(Math.abs(RobotMap.CONTROLLER.getY(Hand.kLeft)), 2)
				* Math.signum(RobotMap.CONTROLLER.getY(Hand.kLeft)) * RobotMap.DriveMap.MAX_SPEED;
		if (Math.abs(xSpeed) < (0.2 * RobotMap.DriveMap.MAX_SPEED)) {
			xSpeed = 0;
		}

		// Get the y speed or sideways/strafe speed. We are inverting this because
		// we want a positive value when we pull to the left. Xbox controllers
		// return positive values when you pull to the right by default.
		double ySpeed = -Math.pow(Math.abs(RobotMap.CONTROLLER.getX(Hand.kLeft)), 2)
				* Math.signum(RobotMap.CONTROLLER.getX(Hand.kLeft)) * RobotMap.DriveMap.MAX_SPEED;
		if (Math.abs(ySpeed) < (0.2 * RobotMap.DriveMap.MAX_SPEED)) {
			ySpeed = 0;
		}
		// Get the rate of angular rotatpion. We are inverting this because we want a
		// positive value when we pull to the left (remember, CCW is positive in
		// mathematics). Xbox controllers return positive values when you pull to
		// the right by default.
		double turnSpeed = robotAnglePID.calculate(getAngle().getDegrees(), angle);
		drive(xSpeed, ySpeed, turnSpeed, fieldRelative);
	}

	public void updateModuleAngles() {
		m_backLeft.readAngle();
		m_backRight.readAngle();
		m_frontLeft.readAngle();
		m_frontRight.readAngle();
	}
}