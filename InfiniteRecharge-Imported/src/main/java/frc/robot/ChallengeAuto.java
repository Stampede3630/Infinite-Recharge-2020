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

    public void gsBPeriodic(){

        //0.36, 2.26
        switch(pathStep)
        {
            case 0:
                Chooser.getInstance().autoChooser(Chooser.RobotState.GALACTIC_SEARCH);
                System.out.println("TX = " + Limelight.getTX());
                if(Limelight.getTX() < 0) {
                    pathStep = 1;
                }
                else if(Limelight.getTX() > 0) {
                    pathStep = 2;
                }
                else {
                    System.out.println(">>>>>> No update due to Limelight");
                }
            break;

            case 1: //RUN BLUE TRAJ
            TrajectoryContainer.getInstance().trajectoryFollowingGSBBlue.auto();
            System.out.println("BLUE TRAJ!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
            break;

            case 2: //RUN RED TRAJ
            TrajectoryContainer.getInstance().trajectoryFollowingGSBRed.auto();
            System.out.println("RED TRAJ+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
            break;

            default:
            System.out.println("default==================================");
            break;



        }
        
    }

    public void resetPathStep(){
        pathStep = 0;
    }
    
    public void galacticSearchPeriodic(){
        switch(pathStep)
        {
            case 0:
                Chooser.getInstance().autoChooser(Chooser.RobotState.GALACTIC_SEARCH);
                System.out.println("TX IS " + Limelight.getTX());
                if((Limelight.getTX() < 0) && (Limelight.getTX() > -12)) {
                    System.out.println("TX IS " + Limelight.getTX());
                    pathStep = 1; //GSA BLUE -8
                }
                else if((Limelight.getTX() > 0) && (Limelight.getTX() < 10)) {
                    System.out.println("TX IS " + Limelight.getTX()); 

                    pathStep = 3; //B BLUE +3
                }/*
                else if(Limelight.getTX() < 0) {
                    System.out.println("TX IS " + Limelight.getTX());

                    pathStep = 2; //GSA RED 0 ------15
                }
                else if(Limelight.getTX() > 0) {
                    System.out.println("TX IS " + Limelight.getTX());

                    pathStep = 4; //GSB RED 18 ++++15
                }*/
                else {
                    System.out.println(">>>>>> No update due to Limelight");
                }
            break;

            case 1: //RUN A BLUE TRAJ
            TrajectoryContainer.getInstance().trajectoryFollowingGSABlue.auto();
            System.out.println("AAAAAA BLUE TRAJ!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
            break;

            case 2: //RUN A RED TRAJ
            TrajectoryContainer.getInstance().trajectoryFollowingGSARed.auto();
            System.out.println("AAAAAAA RED TRAJ+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
            break;

            case 3: //RUN B BLUE TRAJ
            TrajectoryContainer.getInstance().trajectoryFollowingGSBBlue.auto();
            System.out.println("BBBBBBB BLUE TRAJ!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!");
            break;

            case 4: //RUN B RED TRAJ
            TrajectoryContainer.getInstance().trajectoryFollowingGSBRed.auto();
            System.out.println("BBBBBBB RED TRAJ+++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++");
            break;

            default:
            System.out.println("default==================================");
            break;



        }

    }
}
