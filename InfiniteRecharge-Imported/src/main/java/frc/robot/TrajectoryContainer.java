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

	private ProfiledPIDController thetaController = new ProfiledPIDController(
			RobotMap.AutoConstants.KP_THETA_CONTROLLER, 0, 0, RobotMap.AutoConstants.kThetaControllerConstraints);
	/*
	 * private Trajectory traj = TrajectoryGenerator.generateTrajectory(new
	 * Pose2d(0, 0, new Rotation2d(0)), //List.of(new Translation2d(1, 1), new
	 * Translation2d(2, -1)), new Pose2d(3, 0, new Rotation2d(0)), config);
	 * //List.of(new Translation2d(0.5, 0)), new Pose2d(1, 0, new Rotation2d(0)),
	 * config); List.of(new Translation2d(0, 0.5)), new Pose2d(0, 1, new
	 * Rotation2d(Math.PI/6)), config);
	 */
	private Trajectory traj2 = TrajectoryGenerator.generateTrajectory(new Pose2d(3.05, -2.4, new Rotation2d(0)), // SETPOINTS
																													// FOR
																													// X
																													// INVERTED
			List.of(new Translation2d(6.166, -.704), // Trench ball 1
					new Translation2d(7.08, -.704)), // Trench ball 2
			// new Translation2d(7.99, .704)),//Trench ball 3
			new Pose2d(7.99, -.704, new Rotation2d(0)), config); // intake logic path 2*/

	private Trajectory traj4 = TrajectoryGenerator.generateTrajectory(
			List.of(new Pose2d(3.05, -2.4, new Rotation2d(0)), new Pose2d(6.166, -.704, new Rotation2d(0)),
					// new Pose2d( 7.08, -.704, new Rotation2d(0) ),
					new Pose2d(7.99, -.704, new Rotation2d(0))),
			config);
	/*
	 * 
	 * private Trajectory traj3 = TrajectoryGenerator.generateTrajectory(new Pose2d
	 * ( 8.21,3.05, new Rotation2d(0)), List.of( new Translation2d(6.23,
	 * 6.36),//Trench 2 ball 1 new Translation2d(7.74, 6.36)),//Trench 2 ball 2
	 * //new Translation2d(4.08, 3.05)),//back to line new Pose2d(4.08, 3.05, new
	 * Rotation2d(45)), config); //intake logic path 2
	 * 
	 */

	/*
	 * OLD TRIES AT SLALOM // x does not like negatives, private Trajectory
	 * slalomTraj1 = TrajectoryGenerator.generateTrajectory( List.of( new Pose2d(.3,
	 * .3, new Rotation2d(0)), new Pose2d(.9, 0, new Rotation2d(0)), new Pose2d(.9,
	 * .6, new Rotation2d(0)), new Pose2d(1.8, 1.05, new Rotation2d(0)) ), config);
	 * 
	 * private Trajectory slalomTraj2 = TrajectoryGenerator.generateTrajectory( new
	 * Pose2d(0, 0, new Rotation2d(0)), // x is long, y is short List.of( new
	 * Translation2d(.1,.4), //cross new Translation2d(.3, .5), new
	 * Translation2d(.7, 1.4), new Translation2d(2.3, 2),// cross 2 new
	 * Translation2d(3, 3.8), new Translation2d(1.5, 4), new Translation2d(2.3,
	 * 2.5), new Translation2d(3, 2.2), new Translation2d(2.5, -.3)
	 * 
	 * ), new Pose2d(0,0 , new Rotation2d(0)), //new Pose2d(-1, -2.5, new
	 * Rotation2d(0)), config );
	 */

	private Trajectory slalomTraj = TrajectoryGenerator.generateTrajectory(new Pose2d(0.72, 0.75, new Rotation2d(0)),
			List.of(new Translation2d(2.12, 0.97), new Translation2d(2.38, 2.2), new Translation2d(6.20, 2.1),
					new Translation2d(6.51, 0.45), new Translation2d(7.8, 0.42), new Translation2d(7.82, 1.90),
					new Translation2d(5.35, 1.90), new Translation2d(5.1, 0.5), new Translation2d(3, 0.38),
					new Translation2d(.72, 0.5), new Translation2d(.67, 2.05)),
			new Pose2d(.47, 2.28, new Rotation2d(0)),

			// new Pose2d(6.858, 1.524, new Rotation2d(0)),
			config);

	private Trajectory basicdriveback = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)),
			List.of(new Translation2d(1, 0), new Translation2d(2, 0)

			), new Pose2d(2.3, 0, new Rotation2d(0)),

			// new Pose2d(6.858, 1.524, new Rotation2d(0)),
			config);

	private Trajectory ThiefAutoPart1 = TrajectoryGenerator.generateTrajectory(new Pose2d(0, 0, new Rotation2d(0)),
			List.of(new Translation2d(1, 0), new Translation2d(2, 0)

			), new Pose2d(2.05, 0, new Rotation2d(0)),

			// new Pose2d(6.858, 1.524, new Rotation2d(0)),
			config);

	private Trajectory ThiefAutoPart2 = TrajectoryGenerator.generateTrajectory(new Pose2d(1.4, 0, new Rotation2d(0)),
			List.of(new Translation2d(1.2, .4), new Translation2d(1.1, .8), new Translation2d(1, 3),
					new Translation2d(.9, 4.5)

			), new Pose2d(.85, 4.7, new Rotation2d(0)),

			// new Pose2d(6.858, 1.524, new Rotation2d(0)),
			config);

	private Trajectory bounceTraj = TrajectoryGenerator.generateTrajectory(new Pose2d(1.19, 2.29, new Rotation2d(0)),
			List.of(new Translation2d(1.72, 2.25), new Translation2d(2.34, 2.41), new Translation2d(2.22, 3.2), // 1st
					new Translation2d(2.28, 2.8), new Translation2d(2.71, 1.7), new Translation2d(3.08, 0.7),
					new Translation2d(4.1, 0.95), new Translation2d(4.2, 2.7), new Translation2d(4.23, 3.95), // 2nd
					new Translation2d(4.05, 2.5), new Translation2d(4.45, 1.45), new Translation2d(5.6, 1.25),
					new Translation2d(6.25, 1.8), new Translation2d(6.45, 2.79), new Translation2d(6.6, 4.55), // 3rd
					new Translation2d(6.7, 3.85)),
			new Pose2d(8.75, 3.85, new Rotation2d(0)), config);
	private Trajectory oldbarrelRollTraj = TrajectoryGenerator.generateTrajectory(
			new Pose2d(0.65, 1.94, new Rotation2d(0)),
			List.of(new Translation2d(2.16, 2.16), new Translation2d(3.45, 1.94), new Translation2d(4.43, 1.30),
					new Translation2d(3.67, 0.0), new Translation2d(2.27, 1.08), new Translation2d(3.45, 2.16),
					new Translation2d(6.26, 1.73), new Translation2d(6.48, 3.45), new Translation2d(4.75, 3.02),
					new Translation2d(5.18, .76), new Translation2d(8.20, .22), new Translation2d(7.99, 1.94),
					new Translation2d(6.26, 1.94), new Translation2d(3.45, 2.16)),
			new Pose2d(0.65, 2.16, new Rotation2d(0)), config);

	private Trajectory BarrelRollTraj = TrajectoryGenerator.generateTrajectory(
			new Pose2d(1.19, 2.12, new Rotation2d(0)),
			List.of(new Translation2d(2.13, 2.11), new Translation2d(3.19, 1.69), // T SATRT 1
					new Translation2d(4.29, 0.80), // L
					new Translation2d(3.01, -1.0), // B
					new Translation2d(1.65, 0.78), // R
					new Translation2d(3.1, 1.75), // T END 1
					new Translation2d(5.85, .90), // B START 2
					new Translation2d(6.1, 2.40), // T
					new Translation2d(3.35, 2.40), // L END 2
					new Translation2d(4.23, 0.3), // L START 3
					new Translation2d(6.6, -0.47), // B
					new Translation2d(6.8, 1.4), // T
					new Translation2d(5.57, 1.90), // L END 3
					new Translation2d(1.73, 1.94)// BACK
			),

			new Pose2d(-1, 1.98, new Rotation2d(0)), config);

	// GALACTIC SEARCH TRAJECTORIES
	private Trajectory gsARedTrajQuintic = TrajectoryGenerator.generateTrajectory( // map A, red path
			List.of(new Pose2d(0.25, 2.54, new Rotation2d(0.00)), new Pose2d(2.29, 2.29, new Rotation2d(30.00)),
					new Pose2d(3.81, 1.52, new Rotation2d(-30.00)), new Pose2d(4.57, 3.81, new Rotation2d(-25.00)),
					new Pose2d(8.89, 2.5, new Rotation2d(0.00))),
			config);

	private Trajectory gsARedTraj = TrajectoryGenerator.generateTrajectory(
			new Pose2d(1, 2.2, new Rotation2d(0)), List.of(new Translation2d(2.7, 1.75), new Translation2d(3.23, 1.65),
					new Translation2d(3.9, 3.8), new Translation2d(8.00, 4.0)),
			new Pose2d(10.0, 3.9, new Rotation2d(0)), config);

	private Trajectory gsABlueTraj = TrajectoryGenerator.generateTrajectory( // SUBTRACT 30 IN OFF OF THESE !!!!!!1
			new Pose2d(.4, 1.2, new Rotation2d(0)),
			List.of(new Translation2d(1.88, 1.91), new Translation2d(2.90, 0.51), new Translation2d(4.04, 0.51),
					new Translation2d(4.78, 0.51), new Translation2d(3.94, 2.60), new Translation2d(5.33, 2.45),
					new Translation2d(6.35, 1.90)),
			new Pose2d(8.71, 1.85, new Rotation2d(0)), config);

	private Trajectory gsBRedTraj = TrajectoryGenerator.generateTrajectory(new Pose2d(1, 2.8, new Rotation2d(0)),
			List.of(new Translation2d(1.73, 2.6), new Translation2d(2.5, 2.5), new Translation2d(2.8, 1.4),
					new Translation2d(3.9, 1.65), new Translation2d(4.3, 2.8), new Translation2d(5.71, 2.9)),
			new Pose2d(9.89, 2.9, new Rotation2d(0)), config);
	private Trajectory gsBBlueTraj = TrajectoryGenerator.generateTrajectory(new Pose2d(.4, 1.2, new Rotation2d(0)),
			List.of(new Translation2d(2.69, 1.1), new Translation2d(4.06, .97), // first ball
					new Translation2d(4.2, 1.63 - .30 - .30), new Translation2d(4, 2.90), new Translation2d(5, 2.95),
					new Translation2d(5.5, 1.63 - .30 - .30), // second ball
					new Translation2d(8.13 - 1, 1.52 - .30 - .30)),
			new Pose2d(8.76, 1.50 - .30, new Rotation2d(0)), config);

	private Trajectory goBallTraj = TrajectoryGenerator.generateTrajectory(new Pose2d(2.58, 2.02, new Rotation2d(0)),
			List.of(new Translation2d(4.38, 3.65), new Translation2d(5.5, 3.7), new Translation2d(6.21, 3.7)),
			new Pose2d(7, 3.6, new Rotation2d(0)), config);

	private Trajectory goLineUpTraj = TrajectoryGenerator.generateTrajectory(

			new Pose2d(7, 3.6, new Rotation2d(0)),
			List.of(new Translation2d(6.21, 3.7), new Translation2d(5.5, 3.7), new Translation2d(4.38, 3.65)),
			new Pose2d(2.58, 2.02, new Rotation2d(0)), config);

	// CHANGE STARTING CRDS !!!!!!!!!!!!!!!!
	public TrajectoryFollowing trajectoryFollowingBarrelRoll = new TrajectoryFollowing(BarrelRollTraj, xController,
			yController, thetaController);
	public TrajectoryFollowing trajectoryFollowingbasicdriveback = new TrajectoryFollowing(basicdriveback, xController,
			yController, thetaController);
	public TrajectoryFollowing trajectoryFollowingThiefAutoPart1 = new TrajectoryFollowing(ThiefAutoPart1, xController,
			yController, thetaController);
	public TrajectoryFollowing trajectoryFollowingThiefAutoPart2 = new TrajectoryFollowing(ThiefAutoPart2, xController,
			yController, thetaController);
	public TrajectoryFollowing trajectoryFollowingGSARed = new TrajectoryFollowing(gsARedTraj, xController, yController,
			thetaController);
	public TrajectoryFollowing trajectoryFollowingGSABlue = new TrajectoryFollowing(gsABlueTraj, xController,
			yController, thetaController);
	public TrajectoryFollowing trajectoryFollowingGSBRed = new TrajectoryFollowing(gsBRedTraj, xController, yController,
			thetaController);
	public TrajectoryFollowing trajectoryFollowingGSBBlue = new TrajectoryFollowing(gsBBlueTraj, xController,
			yController, thetaController);
	public TrajectoryFollowing trajectoryFollowingSlalom = new TrajectoryFollowing(slalomTraj, xController, yController,
			thetaController);
	public TrajectoryFollowing trajectoryFollowingBounce = new TrajectoryFollowing(bounceTraj, xController, yController,
			thetaController);
	public TrajectoryFollowing trajectoryFollowing = new TrajectoryFollowing(traj4, xController, yController,
			thetaController);
	public TrajectoryFollowing trajectoryFollowingGoBall = new TrajectoryFollowing(goBallTraj, xController, yController,
			thetaController);
	public TrajectoryFollowing trajectoryFollowingGoLineUp = new TrajectoryFollowing(goLineUpTraj, xController,
			yController, thetaController);

	public void resetTimer() {
		TrajectoryFollowing.m_timer.reset();
	}

}