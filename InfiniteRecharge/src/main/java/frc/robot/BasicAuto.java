/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class BasicAuto {
    private Timer autoTime = new Timer();
    private double timeThreshold = 8;
    private double shooterWait = 0;
    private PIDController autoDistancePID = new PIDController(1, 0, 0);
    private double meterSetpoint = -1;
    private int step = 0;
    private int trajStep = 0;
  
    public void resetAutoTime()
    {
        autoTime.reset();
        autoTime.start();
    }


    public void trajectoryPeriodic()
    {
        switch(trajStep)
        {
            case 0:
            RobotMap.setDriveTalonsBrake();
		    RobotMap.resetEncoders();
		    TrajectoryContainer.getInstance().trajectoryFollowing.resetAll();
		    resetAutoTime();
            RobotMap.AutoBooleans.TRAJECTORY_DONE = false;
            //Chooser.getInstance().autoChooser(Chooser.RobotState.INITIATION_LINE_SHOT);
            trajStep = 2;//++;
            break;
            
            case 2:
            TrajectoryContainer.getInstance().trajectoryFollowing.auto();
            if(RobotMap.AutoBooleans.TRAJECTORY_DONE)
            {   
                resetAutoTime();
                RobotMap.AutoBooleans.INTAKE_NOW = false;
                //Chooser.getInstance().autoChooser(Chooser.RobotState.SHORT_TRENCH);
                RobotMap.AutoBooleans.SHOOT_NOW = true;
                trajStep++;
            }
            break;

        }

    }
     
}
