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
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;

/**
 * Add your docs here.
 */
public class TrajectoryContainer
{
    Robot robot;
    private PIDController xController = new PIDController(AutoConstants.kPXController, 0, 0);
    private PIDController yController = new PIDController(AutoConstants.kPYController, 0, 0);
    private Drivetrain drive = Drivetrain.getInstance();
    private TrajectoryConfig config = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
    AutoConstants.kMaxAccelerationMetersPerSecondSquared)
// Add kinematics to ensure max speed is actually obeyed
.setKinematics(drive.m_kinematics);
    
    private ProfiledPIDController thetaController = new ProfiledPIDController(AutoConstants.kPThetaController, 0, 0, AutoConstants.kThetaControllerConstraints);
    private Trajectory traj = TrajectoryGenerator.generateTrajectory(
        new Pose2d(0,0, 
        new Rotation2d(0)), 
        List.of(
            new Translation2d(1, 0)//, 
            //new Translation2d(2,-1)
            ), 
        new Pose2d(2,0,new Rotation2d(0)), 
        config
        );

    
    public TrajectoryFollowing trajectoryFollowing = new TrajectoryFollowing(drive, traj, xController, yController, thetaController);
    
    
}