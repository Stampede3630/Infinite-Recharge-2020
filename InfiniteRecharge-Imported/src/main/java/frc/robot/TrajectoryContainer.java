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
    /*
    private Trajectory traj = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)),
			//List.of(new Translation2d(1, 1), new Translation2d(2, -1)), new Pose2d(3, 0, new Rotation2d(0)), config);
			//List.of(new Translation2d(0.5, 0)), new Pose2d(1, 0, new Rotation2d(0)), config);
            List.of(new Translation2d(0, 0.5)), new Pose2d(0, 1, new Rotation2d(Math.PI/6)), config);
            */
	private Trajectory traj2 = TrajectoryGenerator.generateTrajectory(new Pose2d (3.05, -2.4, new Rotation2d(0)),   //SETPOINTS FOR X INVERTED
	List.of(
		new Translation2d(6.166,-.704),//Trench ball 1
		new Translation2d(7.08,-.704)),//Trench ball 2
		//new Translation2d(7.99, .704)),//Trench ball 3
        new Pose2d(7.99, -.704, new Rotation2d(0)), config); //intake logic path 2*/

	private Trajectory traj4 =  TrajectoryGenerator.generateTrajectory(
	List.of(
		new Pose2d (3.05, -2.4, new Rotation2d(0)),
		new Pose2d( 6.166,-.704, new Rotation2d(0)),
		//new Pose2d( 7.08, -.704, new Rotation2d(0) ),
		new Pose2d( 7.99, -.704, new Rotation2d(0))), config);
     /*

    private Trajectory traj3 = TrajectoryGenerator.generateTrajectory(new Pose2d ( 8.21,3.05, new Rotation2d(0)),
	List.of(
        new Translation2d(6.23, 6.36),//Trench 2 ball 1
		new Translation2d(7.74, 6.36)),//Trench 2 ball 2
		//new Translation2d(4.08, 3.05)),//back to line
        new Pose2d(4.08, 3.05, new Rotation2d(45)), config); //intake logic path 2
        
	*/
	// x does not like negatives, 
	private Trajectory slalomTraj1 = TrajectoryGenerator.generateTrajectory(
		List.of(
			new Pose2d(.3, .3, new Rotation2d(0)),
			new Pose2d(.9, 0, new Rotation2d(0)),
			new Pose2d(.9, .6, new Rotation2d(0)),
			new Pose2d(1.8, 1.05, new Rotation2d(0))
			), config);
	
	private Trajectory slalomTraj2 = TrajectoryGenerator.generateTrajectory(
		new Pose2d(0, 0, new Rotation2d(0)),
		// x is long, y is short
		List.of(
			new Translation2d(.1,.4), //cross
			new Translation2d(.3, .5),
			new Translation2d(.7, 1.4),
			new Translation2d(2.3, 2),// cross 2
			new Translation2d(3, 3.8),
			new Translation2d(1.5, 4),
			new Translation2d(2.3, 2.5),
			new Translation2d(3, 2.2),
			new Translation2d(2.5, -.3)
			
		),
		new Pose2d(0,0 , new Rotation2d(0)),
		//new Pose2d(-1, -2.5, new Rotation2d(0)),
		config
	);

	private Trajectory slalomTraj = TrajectoryGenerator.generateTrajectory(
		new Pose2d(0.70, 0.75, new Rotation2d(0)),
		List.of(
			new Translation2d(2.08, 0.97),
			new Translation2d(2.56, 1.98),
			new Translation2d(6.21, 1.90),
			new Translation2d(6.42, 0.63),
			new Translation2d(7.67, 0.60),
			new Translation2d(7.70, 2.13),
			new Translation2d(6.45, 2.15),
			new Translation2d(6.19, 0.63),
			new Translation2d(4.12, 0.42),
			new Translation2d(1.84, 0.72),
			new Translation2d(1.64, 2.00)
		),
		new Pose2d(0.39, 2.28, new Rotation2d(0)),

		// new Pose2d(6.858, 1.524, new Rotation2d(0)),
		config
	);

	private Trajectory bounceTraj = TrajectoryGenerator.generateTrajectory(
		new Pose2d(0, 0, new Rotation2d(0)),

		List.of(
			new Translation2d(0, .3),
			new Translation2d(0, .4)
		),
		new Pose2d(0, .5, new Rotation2d(0)),
		config
	);

	private Trajectory testTraj = TrajectoryGenerator.generateTrajectory(
		new Pose2d(0, 0, new Rotation2d(0)),

		List.of(
			new Translation2d(0, .2),/*
			new Translation2d(0, .3),
			new Translation2d(0, .4),
			new Translation2d(0, .5),
			new Translation2d(0, .6),
			new Translation2d(0, .7),
			new Translation2d(0, .8),
			new Translation2d(0, .9),
			new Translation2d(0, 1.0),
			new Translation2d(0, 1.1),
			new Translation2d(0, 1.2),
			new Translation2d(0, 1.3),
			new Translation2d(0, 1.4),*/
			new Translation2d(0, 1.5)
		),
		new Pose2d(0, 2.0, new Rotation2d(0)),
		config
	);

	//CHANGE STARTING CRDS !!!!!!!!!!!!!!!!
	public TrajectoryFollowing trajectoryFollowing = new TrajectoryFollowing(slalomTraj, xController, yController,
			thetaController);

}