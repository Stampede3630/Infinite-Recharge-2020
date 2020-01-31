/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;

public class Hatch implements ManipulatorMode {
    RobotMap robotMap;
    boolean manipulatorOut, isIntaking;
    Manipulator manipulator;
    Timer deployTimer;

    public Hatch(Manipulator manipulator) 
    {
        robotMap = RobotMap.getRobotMap();
        this.manipulator = manipulator;
        manipulatorOut = false;

        deployTimer = new Timer();
    }

    public void endAll()
    {
        if(deployTimer.get() > 1 || deployTimer.get() == 0 || isIntaking)
        {
            deployTimer.stop();
            deployTimer.reset();
            deployTimer.stop();
            robotMap.talonHatchL.set(0);
            robotMap.talonHatchR.set(0);
        }
    }

    public void engage() //slide out
    {
        if(robotMap.bumperR.get())
        {
            manipulatorOut = true;
            robotMap.hatchExtend.set(DoubleSolenoid.Value.kReverse);
        }
    }

    public void intake() 
    {
        if(robotMap.getTrigger()>0.2&&!(robotMap.hatchButton.get() && robotMap.dumbHatchButton.get()))
        {
            robotMap.talonHatchR.set(-0.5);
            robotMap.talonHatchL.set(0.5);
            isIntaking = true;
            
        }
        else
        {
            robotMap.talonHatchR.set(0);
            robotMap.talonHatchL.set(0);
            isIntaking = false;
            
        }

    }

    public void deploy(boolean rocketMode) //left trigger
    {
        if(robotMap.getTrigger()<-0.2)
        {
            robotMap.talonHatchR.set(0.8);
            robotMap.talonHatchL.set(-0.8);
            isIntaking = false;
            if(deployTimer.get() == 0)
            {
                deployTimer.start();
            }
        }

        
        
/*       else
        {
            robotMap.talonHatchR.set(0);
            robotMap.talonHatchL.set(0);
        }
*/

        /*if(robotMap.getTrigger()<=-0.75)
        {
            circleTimer.start();
            robotMap.hatchDeploy.set(DoubleSolenoid.Value.kForward);
            robotMap.hatchExtend.set(DoubleSolenoid.Value.kForward);
        }
        if(circleTimer.get()>=1)
        {
            circleTimer.stop();
            circleTimer.reset();
            robotMap.hatchDeploy.set(DoubleSolenoid.Value.kReverse);
        }*/
    }

    public void disengage() //slide in
    {
        if(robotMap.bumperL.get() && (!robotMap.hatchButton.get()) && !robotMap.dumbHatchButton.get())
        {
            robotMap.hatchExtend.set(DoubleSolenoid.Value.kForward);
        }
    }

    public void intakeAuto()
    {

    }

    public void deployAuto(double robotTime)
    {
        if(!(DriverStation.getInstance().getMatchTime()>=robotTime+2))
        {
            robotMap.talonHatchR.set(0.8);
            robotMap.talonHatchL.set(-0.8);
        }
        else
        {
            robotMap.talonHatchR.set(0);
            robotMap.talonHatchL.set(0);
            Constants.autoDeploy = false;
        }
    }

}
