/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;

/**
 * Add your docs here.
 */
public class TrajectoryContainer {
	// private Robot robot;
	private PIDController xController = new PIDController(RobotMap.AutoConstants.KPX_CONTROLLER, 0, 0);
	private PIDController yController = new PIDController(RobotMap.AutoConstants.KPY_CONTROLLER, 0, 0);
	private TrajectoryConfig config = new TrajectoryConfig(RobotMap.AutoConstants.MAX_SPEED_METERS_PER_SECOND,
			RobotMap.AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
					// Add kinematics to ensure max speed is actually obeyed
					.setKinematics(Drivetrain.m_kinematics);

	private ProfiledPIDController thetaController = new ProfiledPIDController(RobotMap.AutoConstants.KP_THETA_CONTROLLER, 0, 0,
	RobotMap.AutoConstants.kThetaControllerConstraints);
	private Trajectory traj = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)),
			//List.of(new Translation2d(1, 1), new Translation2d(2, -1)), new Pose2d(3, 0, new Rotation2d(0)), config);
			//List.of(new Translation2d(0.5, 0)), new Pose2d(1, 0, new Rotation2d(0)), config);
			List.of(new Translation2d(0, 0.5)), new Pose2d(0, 1, new Rotation2d(0)), config);

	public TrajectoryFollowing trajectoryFollowing = new TrajectoryFollowing(traj, xController, yController,
			thetaController);

}