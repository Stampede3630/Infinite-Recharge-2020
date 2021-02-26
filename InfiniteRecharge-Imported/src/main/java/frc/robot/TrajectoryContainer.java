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

	/* OLD TRIES AT SLALOM
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
	); */

	private Trajectory slalomTraj = TrajectoryGenerator.generateTrajectory(
		new Pose2d(0.72-.27, 0.75, new Rotation2d(0)),
		List.of(
			new Translation2d(2.12, 1.20),//0.97 CROSS ONE
			new Translation2d(2.61-.27, 1.98),
			new Translation2d(6.35-.27-.3, 1.90), //cross two - beginning of loop
			new Translation2d(6.56-.27-.2, 0.63),
			new Translation2d(7.84-.27-.2, 0.60),
			new Translation2d(7.87-.27-.3, 2.12),
			new Translation2d(6.59-.27-.3, 2.15),//cross three
			new Translation2d(6.21-.27-.3, 0.50),
			new Translation2d(4.22-.27-.3, 0.42),
			new Translation2d(1.88-.27-.4, 0.72), //cross four
			new Translation2d(1.68-.27-.3, 2.00)
		),
		new Pose2d(0.40-.27, 2.00, new Rotation2d(0)), //2.28

		// new Pose2d(6.858, 1.524, new Rotation2d(0)),
		config
	);

	private Trajectory bounceTraj = TrajectoryGenerator.generateTrajectory(
		new Pose2d(0.74, 2.29, new Rotation2d(0)),
		List.of(
			new Translation2d(1.72, 2.29),
			new Translation2d(2.34, 2.41),
			new Translation2d(2.22, 3.81),
			new Translation2d(2.34, 2.41),
			new Translation2d(2.71, 2.29),
			new Translation2d(3.08, 0.89),
			new Translation2d(4.43, 0.89),
			new Translation2d(4.56, 2.03),
			new Translation2d(4.68, 3.81),
			new Translation2d(4.56, 2.03),
			new Translation2d(4.43, 1.27),
			new Translation2d(5.42, 1.02),
			new Translation2d(6.65, 1.27),
			new Translation2d(6.78, 2.79),
			new Translation2d(6.78, 4.06),
			new Translation2d(6.78, 2.79)
		),
		new Pose2d(8.38, 2.79, new Rotation2d(0)),
		config
	);

	private Trajectory barrelRollTraj = TrajectoryGenerator.generateTrajectory(
		new Pose2d(0.76, 2.03, new Rotation2d(0)),
		List.of(
			new Translation2d(2.36, 2.20),
			new Translation2d(3.73, 1.93),
			new Translation2d(4.39, 1.22),
			new Translation2d(3.44, 0.34),
			new Translation2d(2.33, 1.29),
			new Translation2d(3.74, 2.16),
			new Translation2d(6.24, 1.84),
			new Translation2d(6.52, 3.20),
			new Translation2d(4.46, 3.27),
			new Translation2d(5.52, 0.95),
			new Translation2d(7.56, 0.42),
			new Translation2d(7.86, 2.24),
			new Translation2d(6.71, 2.28),
			new Translation2d(3.74, 2.39)
		),
		new Pose2d(0.77, 2.49, new Rotation2d(0)),
		config
	);

	//CHANGE STARTING CRDS !!!!!!!!!!!!!!!!
	public TrajectoryFollowing trajectoryFollowing = new TrajectoryFollowing(bounceTraj, xController, yController,
			thetaController);

}