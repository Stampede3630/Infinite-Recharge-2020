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
	private static TrajectoryContainer instance;

	static {
		instance = new TrajectoryContainer();
	}

	public static TrajectoryContainer getInstance() {
		return instance;
	}

	private TrajectoryContainer() // private to prevent instantiation elsewhere
	{

	}
	
	private PIDController xController = new PIDController(RobotMap.AutoConstants.KPX_CONTROLLER, 0, 0);
	private PIDController yController = new PIDController(RobotMap.AutoConstants.KPY_CONTROLLER, 0, 0);
	private TrajectoryConfig config = new TrajectoryConfig(RobotMap.AutoConstants.MAX_SPEED_METERS_PER_SECOND,
			RobotMap.AutoConstants.MAX_ACCELERATION_METERS_PER_SECOND_SQUARED)
					// Add kinematics to ensure max speed is actually obeyed
					.setKinematics(RobotMap.DrivetrainMap.KINEMATICS);

	private ProfiledPIDController thetaController = new ProfiledPIDController(RobotMap.AutoConstants.KP_THETA_CONTROLLER, 0, 0,
	RobotMap.AutoConstants.kThetaControllerConstraints);
	private Trajectory traj = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)),
			//List.of(new Translation2d(1, 1), new Translation2d(2, -1)), new Pose2d(3, 0, new Rotation2d(0)), config);
			//List.of(new Translation2d(0.5, 0)), new Pose2d(1, 0, new Rotation2d(0)), config);
			List.of(new Translation2d(0, 0.5)), new Pose2d(0, 1, new Rotation2d(Math.PI/6)), config);
	private Trajectory traj2 = TrajectoryGenerator.generateTrajectory(new Pose2d (3.05, -2.4, new Rotation2d(0)),
	List.of(
		new Translation2d(6.166,-.704),//Trench ball 1
		new Translation2d(7.08,-.704)),//Trench ball 2
		//new Translation2d(7.99, .704)),//Trench ball 3
		new Pose2d(7.99, -.704, new Rotation2d(0)), config); //intake logic path 2
	
	public TrajectoryFollowing trajectoryFollowing = new TrajectoryFollowing(traj2, xController, yController,
			thetaController);

}