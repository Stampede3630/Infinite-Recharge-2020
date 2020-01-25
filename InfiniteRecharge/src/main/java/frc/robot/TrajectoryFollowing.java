/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.List;
import java.util.function.Consumer;
import java.util.function.Supplier;
import frc.robot.AutoConstants;
import frc.robot.Drivetrain;
import frc.robot.Drivetrain;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import frc.robot.SwerveControllerCommand;

/**
 * Add your docs here.
 */
public class TrajectoryFollowing {

    private Drivetrain driveBase;
    private TrajectoryConfig config;
    public Trajectory trajectory;
    private final Timer m_timer = new Timer();
    private Pose2d m_finalPose;
    private final SwerveDriveKinematics m_kinematics;
    private final PIDController m_xPIDController;
    private final PIDController m_yPIDController;
    private final ProfiledPIDController m_thetaPIDController;
    private SwerveModuleState[] m_outputModuleStates;
    private final SwerveDriveOdometry m_odometry;
   

    public TrajectoryFollowing(Drivetrain drive, Trajectory traj,
    PIDController xController,
    PIDController yController,
    ProfiledPIDController thetaController)
    {
    driveBase = drive;
    config = new TrajectoryConfig(AutoConstants.kMaxSpeedMetersPerSecond,
                         AutoConstants.kMaxAccelerationMetersPerSecondSquared)
        // Add kinematics to ensure max speed is actually obeyed
        .setKinematics(driveBase.m_kinematics);
    trajectory = traj;
    m_kinematics = driveBase.m_kinematics;
    m_xPIDController = xController;
    m_yPIDController = yController;
    m_thetaPIDController = thetaController;
    m_outputModuleStates = m_kinematics.toSwerveModuleStates(new ChassisSpeeds(0, 0, 0));
    m_odometry = driveBase.m_odometry;
    
    m_finalPose = trajectory.sample(trajectory.getTotalTimeSeconds()).poseMeters;
    m_timer.reset();
    m_timer.start();
    
    }
    
   
    public void auto()
    {
        updateAutoStates();
        driveBase.setModuleStates(m_outputModuleStates);  
    }

    @SuppressWarnings("LocalVariableName")
  public void updateAutoStates() {
    
    double curTime = m_timer.get();

    var desiredState = trajectory.sample(curTime);
    var desiredPose = desiredState.poseMeters;

    var poseError = desiredPose.relativeTo(m_odometry.getPoseMeters()); //get velocity

    double targetXVel = m_xPIDController.calculate(
        m_odometry.getPoseMeters().getTranslation().getX(),
       desiredPose.getTranslation().getX());

    double targetYVel = m_yPIDController.calculate(
        m_odometry.getPoseMeters().getTranslation().getY(),
        desiredPose.getTranslation().getY());

    // The robot will go to the desired rotation of the final pose in the trajectory,
    // not following the poses at individual states.
    double targetAngularVel = m_thetaPIDController.calculate(
        m_odometry.getPoseMeters().getRotation().getRadians(),
        m_finalPose.getRotation().getRadians());

    double vRef = desiredState.velocityMetersPerSecond;

    targetXVel += vRef * poseError.getRotation().getCos();
    targetYVel += vRef * poseError.getRotation().getSin();

    var targetChassisSpeeds = new ChassisSpeeds(targetXVel, targetYVel, targetAngularVel);

    m_outputModuleStates = m_kinematics.toSwerveModuleStates(targetChassisSpeeds);

  }

  public void setTrajectory(Trajectory newTrajectory)
  {
      trajectory = newTrajectory;
      m_finalPose = trajectory.sample(trajectory.getTotalTimeSeconds()).poseMeters;
      m_timer.reset();
      m_timer.start();
  }
}
