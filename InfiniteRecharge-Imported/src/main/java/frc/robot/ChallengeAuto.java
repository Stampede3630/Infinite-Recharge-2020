// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.RobotMap.DriveMap;

/** Add your docs here. */
public class ChallengeAuto {
    private Timer autoTime = new Timer();
    private double timeThreshold = 8;
    private int pathStep = 0;

    public static enum PathState {

        UNKNOWN, RED, BLUE
    }
    private static ChallengeAuto instance;
    
    static {
		instance = new ChallengeAuto();
    }
    
    public static ChallengeAuto getInstance() {
		return instance;
	}

    public void resetAutoTime() {
        autoTime.reset();
        autoTime.start();
    }

    public void gsAPeriodic(){

        switch(pathStep)
        {
            case 0:
                Chooser.getInstance().autoChooser(Chooser.RobotState.GALACTIC_SEARCH);
                if(Limelight.getTX() < -10) {
                    pathStep = 1;
                }
                else if(Limelight.getTX() >-10) {
                    pathStep = 2;
                }
                else {
                    System.out.println(">>>>>> No update due to Limelight");
                }
            break;

            case 1: //RUN BLUE TRAJ
            TrajectoryContainer.getInstance().trajectoryFollowingGSABlue.auto();
            System.out.println("BLUE TRAJ!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
            break;

            case 2: //RUN RED TRAJ
            TrajectoryContainer.getInstance().trajectoryFollowingGSARed.auto();
            System.out.println("RED TRAJ+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
            break;

            default:
            System.out.println("default==================================");
            break;



        }
        
    }
}
