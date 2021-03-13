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
		new Pose2d(0.72, 0.75, new Rotation2d(0)),
		List.of(
			new Translation2d(2.12, 0.97),
			new Translation2d(2.38, 2.11),
			new Translation2d(6.35, 2.03),
			new Translation2d(6.56, 0.63),
			new Translation2d(8.19, 0.59),
			new Translation2d(8.22, 2.12),
			new Translation2d(6.59, 2.15),
			new Translation2d(6.21, 0.63),
			new Translation2d(4.22, 0.42),
			new Translation2d(1.88, 0.72),
			new Translation2d(1.68, 2.00)
		),
		new Pose2d(0.40, 2.28, new Rotation2d(0)),

		// new Pose2d(6.858, 1.524, new Rotation2d(0)),
		config
	);


	private Trajectory bounceTraj = TrajectoryGenerator.generateTrajectory(
		new Pose2d(0.74, 2.29, new Rotation2d(0)),
		List.of(
			new Translation2d(1.72, 2.16),
			new Translation2d(2.34, 2.41),
			new Translation2d(2.22, 3.56),
			new Translation2d(2.34, 2.16),
			new Translation2d(2.71, 2.03),
			new Translation2d(3.08, 0.51),
			new Translation2d(4.43, 0.51),
			new Translation2d(4.80, 2.03),
			new Translation2d(4.93, 3.56),
			new Translation2d(4.80, 2.03),
			new Translation2d(4.80, 0.64),
			new Translation2d(5.42, 0.76),
			new Translation2d(6.90, 0.76),
			new Translation2d(7.15, 2.79),
			new Translation2d(7.15, 3.56),
			new Translation2d(7.15, 2.54)
		),
		new Pose2d(8.75, 2.03, new Rotation2d(0)),
		config
	);
	private Trajectory barrelRollTraj = TrajectoryGenerator.generateTrajectory(
		new Pose2d(0.65, 1.94, new Rotation2d(0)),
		List.of(
			new Translation2d(2.16, 2.16),
			new Translation2d(3.45, 1.94),
			new Translation2d(4.43, 1.30),
			new Translation2d(3.67, 0.0),
			new Translation2d(2.27, 1.08),
			new Translation2d(3.45, 2.16),
			new Translation2d(6.26, 1.73),
			new Translation2d(6.48, 3.45),
			new Translation2d(4.75, 3.02),
			new Translation2d(5.18, .76),
			new Translation2d(8.20, .22),
			new Translation2d(7.99, 1.94),
			new Translation2d(6.26, 1.94),
			new Translation2d(3.45, 2.16)
		),
		new Pose2d(0.65, 2.16, new Rotation2d(0)),
		config
	);

	// GALACTIC SEARCH TRAJECTORIES
	private Trajectory gsARedTrajQuintic = TrajectoryGenerator.generateTrajectory( //map A, red path
		List.of(
			new Pose2d(0.25, 2.54, new Rotation2d(0.00)),
			new Pose2d(2.29, 2.29, new Rotation2d(30.00)),
			new Pose2d(3.81, 1.52, new Rotation2d(-30.00)),
			new Pose2d(4.57, 3.81, new Rotation2d(-25.00)),
			new Pose2d(8.89, 2.5, new Rotation2d(0.00))
		),
		config
	);

	private Trajectory gsARedTraj = TrajectoryGenerator.generateTrajectory(
		new Pose2d(0.25, 2.29, new Rotation2d(0) ),
		List.of(
			new Translation2d(2.72, 2.08 ),
			new Translation2d(3.23, 1.50 ),
			new Translation2d(4.44, 1.60 ),
			new Translation2d(3.96, 3.66 ),
			new Translation2d(8.00, 3.00 )
		),
		new Pose2d(8.76, 2.84, new Rotation2d(0)),
		config
	);


	private Trajectory gsABlueTraj = TrajectoryGenerator.generateTrajectory( //SUBTRACT 30 IN OFF OF THESE !!!!!!1
		new Pose2d(0.25, 2.29, new Rotation2d(0)),
		List.of( 
			new Translation2d(1.88, 1.91),
			new Translation2d(2.90, 0.51),
			new Translation2d(4.04, 0.51), // ball one
			new Translation2d(4.78, 0.51),
			new Translation2d(3.94, 2.29), //ball two
			new Translation2d(5.33, 2.29),
			new Translation2d(6.35, 1.70)
		),
		new Pose2d(8.71, 1.78, new Rotation2d(0)),
		config
	);


	private Trajectory gsBRedTraj = TrajectoryGenerator.generateTrajectory(
		new Pose2d(0.36, 2.26, new Rotation2d(0)),
		List.of(
			new Translation2d(1.73, 3.00),
			new Translation2d(2.64, 2.97),
			new Translation2d(3.38, 1.57),
			new Translation2d(4.34, 1.70),
			new Translation2d(4.90, 2.95),
			new Translation2d(5.71, 3.00)
		),
		new Pose2d(8.89, 2.92, new Rotation2d(0)),
		config
	);

	private Trajectory gsBBlueTraj = TrajectoryGenerator.generateTrajectory(
		new Pose2d(0.36, 2.26, new Rotation2d(0)),
		List.of(
			new Translation2d(2.69, 2.24),
			new Translation2d(4.06, 1.57),
			new Translation2d(5.00, 1.63),
			new Translation2d(5.69, 2.90),
			new Translation2d(6.55, 2.95),
			new Translation2d(7.14, 1.63),
			new Translation2d(8.13, 1.52)
		),
		new Pose2d(8.76, 1.50, new Rotation2d(0)),
		config
	);

	//CHANGE STARTING CRDS !!!!!!!!!!!!!!!!
	public TrajectoryFollowing trajectoryFollowing = new TrajectoryFollowing(gsABlueTraj, xController, yController,
			thetaController);
	public TrajectoryFollowing trajectoryFollowingGSARed = new TrajectoryFollowing(gsARedTraj, xController, yController,
			thetaController);
	public TrajectoryFollowing trajectoryFollowingGSABlue = new TrajectoryFollowing(gsABlueTraj, xController, yController,
			thetaController);		
	public TrajectoryFollowing trajectoryFollowingGSBRed = new TrajectoryFollowing(gsBRedTraj, xController, yController,
			thetaController);		
	public TrajectoryFollowing trajectoryFollowingGSBBlue = new TrajectoryFollowing(gsBBlueTraj, xController, yController,
			thetaController);

	public void resetTimer(){
		TrajectoryFollowing.m_timer.reset();
	}


}