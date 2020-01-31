/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DigitalInput;

public class Ball implements ManipulatorMode {
    
    RobotMap robotMap;
    Manipulator manipulator;
    boolean sensorIntake, manualIntake, gotBall;
    Timer intakeTimer;

    public Ball (Manipulator manipulator) 
    {
        robotMap = RobotMap.getRobotMap();
        this.manipulator = manipulator;
        sensorIntake = true;
        manualIntake = false;
        intakeTimer = new Timer();
        gotBall = false;
    }

    public void endAll()
    {
        robotMap.talonBallIntake.set(0);
        robotMap.talonBallShooter.set(0);
        gotBall = false;
    }

    public void engage () 
    {
        if(robotMap.bumperR.get())
        {
            robotMap.talonBallIntake.set(-0.7);
        }
/*        else
        {
            robotMap.talonBallIntake.set(0);
        }
        */
    }

    public void intake () //right trigger
    {
/*        if(robotMap.getTrigger()>0.2&&robotMap.ballStop.getVoltage()<4.0)
        {
            sensorIntake = false;
            manualIntake = true;
            robotMap.talonBallIntake.set(-0.8);
            robotMap.talonBallShooter.set(1);
        }
        else if(robotMap.ballStop.getVoltage()>4.0&&manualIntake)
        {
            sensorIntake = true;
            manualIntake = false;
            intakeTimer.start();
        }
        else if(intakeTimer.get()<0.6&&sensorIntake)
        {
            robotMap.talonBallIntake.set(-0.8);
            robotMap.talonBallShooter.set(1);
        }
        else 
        {
            manualIntake = false;
            sensorIntake = false;
            intakeTimer.stop();
            intakeTimer.reset();
            robotMap.talonBallIntake.set(0);
            robotMap.talonBallShooter.set(0);
        }
        */
     
        if(robotMap.getTrigger()>0.2&&robotMap.ballButton.get() == true && Constants.ballBottom && robotMap.lowBallButton.get() && !gotBall) 
        {
            robotMap.talonBallIntake.set(-0.55);
            robotMap.talonBallShooter.set(0);
 
            //System.out.println("Ball is going through the bottom");
        }
//        else if (robotMap.getTrigger()>0.2&&robotMap.ballStopBottom.getVoltage()<4.0 && Constants.ballBottom)
        else if(robotMap.getTrigger()>0.2&&robotMap.ballButton.get() == true && Constants.ballTop && !gotBall)
        {
            robotMap.talonBallIntake.set(0);
            robotMap.talonBallShooter.set(0.6);
            //System.out.println("Ball is going through the top");
        }

//new code here
        else if(robotMap.getTrigger()>0.2&&robotMap.ballButton.get() == true && Constants.ballBottom && !robotMap.lowBallButton.get())
        {
            robotMap.talonBallIntake.set(-0.6); //-0.60
            robotMap.talonBallShooter.set(0);
            //System.out.println("Ball is going through the top");
            //System.out.println("Ball has pressed the first button");
        }

        else
        {
            robotMap.talonBallIntake.set(0);
            robotMap.talonBallShooter.set(0);
        }

        if(!robotMap.ballButton.get())
        {
            gotBall = true;
        }


//end


     /*   else if(robotMap.getTrigger()>0.2&&robotMap.ballStopBottom.getVoltage()<4.0)
        {
            robotMap.talonBallIntake.set(1);
            robotMap.talonBallShooter.set(-1);
        }*/
        /*else
        {
            robotMap.talonBallIntake.set(0);
            robotMap.talonBallShooter.set(0);
        } 
        */
    }

    public void deploy (boolean rocketMode) //left trigger
    {
        if(robotMap.getTrigger()<-0.2)
        {
            if(rocketMode)
            {
                robotMap.talonBallShooter.set(-robotMap.getTrigger());
                robotMap.talonBallIntake.set(robotMap.getTrigger());
            }
            else
            {
                robotMap.talonBallShooter.set(0.55*robotMap.getTrigger());
                robotMap.talonBallIntake.set(-0.3);
            }
        }   
        /*else
        {
            robotMap.talonBallIntake.set(0);
            robotMap.talonBallShooter.set(0);
        }
        */
    }

    public void disengage () 
    {
        if(robotMap.bumperL.get())
        {
            robotMap.talonBallIntake.set(0.7);
        }
        /*else
        {
            robotMap.talonBallIntake.set(0);
        }
        */
    }

    public void intakeAuto()
    {
        /*
        if(!robotMap.ballButton.get())
        {
            robotMap.talonBallIntake.set(0);
//            System.out.println("stop intaking");
        }
        else if(Constants.ta>=60||Constants.tv==0)
        {
            robotMap.talonBallIntake.set(-1);
//            System.out.println("intaking");
        }
        */
    }

    public void deployAuto(double robotTime)
    {
        Constants.autoDeploy = false;
    }
}
