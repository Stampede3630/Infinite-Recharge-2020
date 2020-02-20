/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;

/**
 * Add your docs here.
 */
public class TrajectoryFollowing {

	private TrajectoryConfig config;
	public Trajectory trajectory;
	private final Timer m_timer = new Timer();
	private Pose2d m_finalPose;
	private final PIDController m_xController;
	private final PIDController m_yController;
	private final ProfiledPIDController m_thetaController;
	private SwerveModuleState[] m_outputModuleStates;

	public TrajectoryFollowing(Trajectory traj, PIDController xController, PIDController yController,
			ProfiledPIDController thetaController) {
		config = new TrajectoryConfig(RobotMap.AutoConstants.MAX_SPEED_METERS_PER_SECOND,
		RobotMap.AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
						// Add kinematics to ensure max speed is actually obeyed
						.setKinematics(RobotMap.DrivetrainMap.KINEMATICS);
		trajectory = traj;
		m_xController = xController;
		m_yController = yController;
		m_thetaController = thetaController;
		m_outputModuleStates = RobotMap.DrivetrainMap.KINEMATICS.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0));

		m_finalPose = trajectory.sample(trajectory.getTotalTimeSeconds()).poseMeters;
		m_timer.reset();
		m_timer.start();

	}

	public void auto() {
		if(m_timer.get() > trajectory.getTotalTimeSeconds())
		{
			Drivetrain.getInstance().drive(0, 0, 0, false);
		}
		updateAutoStates();
		//System.out.println(m_outputModuleStates[0].speedMetersPerSecond + "," + m_outputModuleStates[0].angle.getDegrees());
		Drivetrain.getInstance().setModuleStates(m_outputModuleStates);

	}

	@SuppressWarnings("LocalVariableName")
	public void updateAutoStates() {

		double curTime = m_timer.get();

		var desiredState = trajectory.sample(curTime);
		var desiredPose = desiredState.poseMeters;

		var poseError = desiredPose.relativeTo(RobotMap.DrivetrainMap.ODOMETRY.getPoseMeters());

		double targetXVel = m_xController.calculate(RobotMap.DrivetrainMap.ODOMETRY.getPoseMeters().getTranslation().getX(),
				desiredPose.getTranslation().getX());

		double targetYVel = m_yController.calculate(RobotMap.DrivetrainMap.ODOMETRY.getPoseMeters().getTranslation().getY(),
				desiredPose.getTranslation().getY());

		// The robot will go to the desired rotation of the final pose in the
		// trajectory,
		// not following the poses at individual states.
		double targetAngularVel = m_thetaController.calculate(RobotMap.DrivetrainMap.ODOMETRY.getPoseMeters().getRotation().getRadians(),
				m_finalPose.getRotation().getRadians());

		double vRef = desiredState.velocityMetersPerSecond;

		targetXVel += vRef * poseError.getRotation().getCos();
		targetYVel += vRef * poseError.getRotation().getSin();
		System.out.println(RobotMap.DrivetrainMap.ODOMETRY.getPoseMeters().getTranslation().getX());
		//System.out.println("xvel: " + targetXVel + ", yvel: " + targetYVel + ", rot: " + targetAngularVel);
		var targetChassisSpeeds = new ChassisSpeeds(targetXVel, targetYVel, targetAngularVel);

		m_outputModuleStates = RobotMap.DrivetrainMap.KINEMATICS.toSwerveModuleStates(targetChassisSpeeds);

	}

	public void setTrajectory(Trajectory newTrajectory) {
		trajectory = newTrajectory;
		m_finalPose = trajectory.sample(trajectory.getTotalTimeSeconds()).poseMeters;
		m_timer.reset();
		m_timer.start();
	}
}
