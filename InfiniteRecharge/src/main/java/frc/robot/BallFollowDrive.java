/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;
import frc.robot.Limelight.Target.TargetType;

/**
 * Add your docs here.
 */
public class BallFollowDrive {

	private static ProfiledPIDController turnPID = new ProfiledPIDController(0.03, 0, 0,
			new TrapezoidProfile.Constraints(RobotMap.PIDConstraints.MAX_ANGULAR_SPEED, Math.PI * 6)); // TODO?
	private static PIDController xPID = new PIDController(0.03, 0, 0);
	private static PIDController yPID = new PIDController(0.03, 0, 0);
	private static PIDController xVelPID = new PIDController(0.03, 0, 0);
	private static PIDController yVelPID = new PIDController(0.03, 0, 0);

	private enum IntakeState {
		Searching, Intaking, Done
	}

	private static IntakeState intakeState;

	private static double lastYAngle;

	private static int lastAngleInvalidTicks;

	private static Drivetrain drivetrain;

	static {
		drivetrain = Drivetrain.getInstance();
	}

	/**
	 * Initializes the LimeLight pipeline to {@value RobotMap#PIPELINE_BALL_FOLLOW}
	 * (and enables vision processing).
	 */
	public static void initLimelight() {
		Limelight.enableVisionProcessing();
		Limelight.setPipeline(RobotMap.Pipelines.BALL_FOLLOW);
	}

	/**
	 * Reset the state.
	 * <p>
	 * (Currently only calls {@link #resetIntakeState()}).
	 */
	public static void reset() {

		resetIntakeState();
	}

	/**
	 * Resets the state of the intake process.
	 * <p>
	 * This sets the state to Searching.
	 * <p>
	 * Call this if the state has been set to Done to start searching again.
	 */
	public static void resetIntakeState() {
		intakeState = IntakeState.Searching;
		lastAngleInvalidTicks = 0;
		lastYAngle = 0;
	}

	/**
	 * Searches for a ball and runs the intake process.
	 * <p>
	 * Currently marks the process as done when it loses sight of the ball at a low
	 * enough Y angle (bottom of the screen).
	 * <p>
	 * This can be reset using {@link #reset() reset()} or
	 * {@link #resetIntakeState() resetIntakeState()}.
	 * 
	 * @return True if the process is considered done.
	 */
	public static boolean intake() {
		Limelight.Target.trackTarget(TargetType.PowerCell);
		double xTargetAngle = Limelight.getTX();

		// System.out.println(xScreenPos);

		boolean hasTarget = Limelight.Target.isValid();
		boolean isInCenter = Math.abs(xTargetAngle) < 16;

		switch (intakeState) {
		case Searching:
			SmartDashboard.putNumber("Intake State", 0);
			break;
		case Intaking:
			SmartDashboard.putNumber("Intake State", 1);
			break;
		case Done:
			SmartDashboard.putNumber("Intake State", 2);
			break;
		}

		SmartDashboard.putNumber("Last Y Angle", lastYAngle);

		if (lastYAngle < -20) {
			lastAngleInvalidTicks = 0;
		} else {
			lastAngleInvalidTicks++;
		}

		switch (intakeState) {
		case Searching:
			if (hasTarget) {
				if (isInCenter) {
					intakeState = IntakeState.Intaking;
					stop();
					lastYAngle = Limelight.getTY();
					break;
				} else {
					drivetrain.drive(0, 0, RobotMap.PIDConstraints.MAX_ANGULAR_SPEED * RobotMap.BALL_FOLLOW.SLOW_SEARCH, false); // TODO Verify rotation speed
				}
			} else {
				drivetrain.drive(0, 0, RobotMap.PIDConstraints.MAX_ANGULAR_SPEED * RobotMap.BALL_FOLLOW.FAST_SEARCH, false); // TODO Verify rotation speed
			}
			break;
		case Intaking:
			if (hasTarget) {
				lastYAngle = Limelight.getTY();
				// stop();
				followTarget(0, 0);
			} else {
				if (lastAngleInvalidTicks < RobotMap.BALL_FOLLOW.FLICKER_PROTECTION) // The ball was last seen in the lower quarter of the screen
				{
					intakeState = IntakeState.Done;
				} else {
					intakeState = IntakeState.Searching;
				}
				stop();
			}
			break;
		case Done:
			stop();
			break;
		}

		return intakeState == IntakeState.Done;
	}

	/**
	 * Stops the robot. Currently only a shortcut for {@link Drivetrain#stop()}.
	 */
	public static void stop() {
		drivetrain.stop();
	}

	/**
	 * Follows the target (ball) at a given offset from the robot.
	 * <p>
	 * Target position is treated in as a top-down view with the robot at the
	 * origin.
	 * <p>
	 * Stops if no target is visible and returns false.
	 * 
	 * @param targetX Target X position (positive to the right). [UNTESTED, MOST
	 *                LIKELY TO THE RIGHT, BLAME THE LAZY DEV]
	 * @param targetY Target Y position (positive away from the robot).
	 * @return True if a target is being followed, false if no target is visible.
	 */
	public static boolean followTarget(double targetX, double targetY) {
		if (!Limelight.Target.isValid()) {
			drivetrain.stop();
			return false;
		}
		double xMotion = 0;
		double yMotion = 0;

		double xDelta = Limelight.Target.getX() - targetX;
		double yDelta = Limelight.Target.getY() - targetY;

		xMotion = -xPID.calculate(xDelta, 0);
		yMotion = yPID.calculate(yDelta, 0);

		// final double POWER = 1;
		//
		// xMotion = Math.pow(Math.abs(xMotion), POWER) * Math.signum(xMotion);
		// yMotion = Math.pow(Math.abs(yMotion), POWER) * Math.signum(yMotion);

		double dist = Math.sqrt(Math.pow(xDelta, 2) + Math.pow(yDelta, 2));

		final double DIST_THRESHOLD = 5; // TODO Basically not used anymore

		if (dist < DIST_THRESHOLD) {
			xMotion *= Math.sqrt((DIST_THRESHOLD - dist) / DIST_THRESHOLD);
			yMotion *= Math.sqrt((DIST_THRESHOLD - dist) / DIST_THRESHOLD);
		}

		xMotion *= -RobotMap.BALL_FOLLOW.FOLLOW_SPEED_MULTIPLIER;
		yMotion *= -RobotMap.BALL_FOLLOW.FOLLOW_SPEED_MULTIPLIER;

		final double DELTA_MULT = 0.5;

		double xVel = xVelPID.calculate(Limelight.Target.getXVel()) * -DELTA_MULT;
		double yVel = yVelPID.calculate(Limelight.Target.getYVel()) * -DELTA_MULT;

		xVel = Math.pow(Math.abs(xVel), 2) * Math.signum(xVel);
		yVel = Math.pow(Math.abs(yVel), 2) * Math.signum(yVel);

		xMotion -= xVel;
		yMotion -= yVel;

		SmartDashboard.putNumber("dist", dist);

		double angleDelta = Math.atan2(xDelta, yDelta) / Math.PI * 180; // TODO Incorporate turnToAngle stuff and all that
		// System.out.println(angleDelta);
		double angleVel = turnPID.calculate(angleDelta, 0);

		// Drivetrain.drive(-MathHelper.remap(xMotion, -1, 1, -Robot.kMaxSpeed,
		// Robot.kMaxSpeed) / 4,
		// MathHelper.remap(yMotion, -1, 1, -Robot.kMaxSpeed, Robot.kMaxSpeed) / 4,
		// MathHelper.remap(angleVel, -1, 1, -Robot.kMaxAngularSpeed,
		// Robot.kMaxAngularSpeed) / 2, false);
		drivetrain.drive(MathHelper.clampUnit(yMotion) * RobotMap.PIDConstraints.MAX_SPEED / 2,
				MathHelper.clampUnit(xMotion) * RobotMap.PIDConstraints.MAX_SPEED / 2,
				MathHelper.clampUnit(angleVel) * RobotMap.PIDConstraints.MAX_ANGULAR_SPEED, false);

		SmartDashboard.putNumber("X Delta", xDelta); // TODO Move into its own SmartDashboard method
		SmartDashboard.putNumber("Y Delta", yDelta);
		SmartDashboard.putNumber("Angle Delta", angleDelta);
		SmartDashboard.putNumber("X Velocity", xMotion);
		SmartDashboard.putNumber("Y Velocity", yMotion);
		SmartDashboard.putNumber("Angular Velocity", angleVel);
		return true;
	}
}