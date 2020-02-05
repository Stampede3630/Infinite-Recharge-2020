package frc.robot.CANLed;

import com.ctre.phoenix.ILoopable;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotMap;

public class LEDMainLoop implements ILoopable
{
    //private RobotMap robotMap = RobotMap.getRobotMap();
    int modulusInt;
    int multiplier = 1;
    DriverStation.Alliance color = DriverStation.getInstance().getAlliance();

    public void onStart()
    {
        Schedulers.PeriodicTasks.start(TaskList.operationRainbowMode);
        //System.out.println("I started :)");
        modulusInt = 0;
    }

    public void onStop()
    {

    }

    public boolean isDone()
    {
        return false;
    }

    public void onLoop()
    {
        modulusInt++;



        if (modulusInt % 8 == 0)
        {
            multiplier = 1;
        }

        else if (modulusInt % 4 == 0)
        {
            multiplier = 0;
        }

     if(!RobotMap.ballButton.get())
        {
            Schedulers.PeriodicTasks.stop(TaskList.operationRainbowMode);
            TaskList.myLEDColorSetter.Hue = 115;
            TaskList.myLEDColorSetter.Saturation = 1;
            TaskList.myLEDColorSetter.Value = 0.4f * multiplier;
        }

        
        else
        {
            Schedulers.PeriodicTasks.start(TaskList.operationRainbowMode);
            TaskList.myLEDColorSetter.Value = 0;
            modulusInt = 0;
        }

    }

}