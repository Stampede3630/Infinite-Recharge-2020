package frc.robot.CANLed;

import com.ctre.phoenix.ILoopable;

import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.RobotMap;

public class LEDMainLoop implements ILoopable
{
    private RobotMap robotMap = RobotMap.getRobotMap();
    int modulusInt;
    int multiplier = 1;
    DriverStation.Alliance color = DriverStation.getInstance().getAlliance();

    public void onStart()
    {
        //Schedulers.PeriodicTasks.start(TaskList.operationRainbowMode);
//        System.out.println("I started :)");
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

        if (modulusInt > 70)
        {
            multiplier = 0;
        }

        else if (modulusInt % 8 == 0)
        {
            multiplier = 1;
        }

        else if (modulusInt % 4 == 0)
        {
            multiplier = 0;
        }

        if(robotMap.hatchButton.get() && robotMap.dumbHatchButton.get())
        {
//            Schedulers.PeriodicTasks.stop(TaskList.operationRainbowMode);
            TaskList.myLEDColorSetter.Hue = 150;
            TaskList.myLEDColorSetter.Saturation = 1;
            TaskList.myLEDColorSetter.Value = 0.4f * multiplier;

        }
        else if(!robotMap.ballButton.get())
        {
//            Schedulers.PeriodicTasks.stop(TaskList.operationRainbowMode);
            TaskList.myLEDColorSetter.Hue = 115;
            TaskList.myLEDColorSetter.Saturation = 1;
            TaskList.myLEDColorSetter.Value = 0.4f * multiplier;
        }
/*        else if(color == DriverStation.Alliance.Red)
        {
            TaskList.myLEDColorSetter.Hue = 120;
            TaskList.myLEDColorSetter.Saturation = 1;
            TaskList.myLEDColorSetter.Value = 0.15f;
            modulusInt = 0;
        }

        else if(color == DriverStation.Alliance.Blue)
        {
            TaskList.myLEDColorSetter.Hue = 280;
            TaskList.myLEDColorSetter.Saturation = 1;
            TaskList.myLEDColorSetter.Value = 0.15f;
            modulusInt = 0;
        }
        */
        else
        {
            TaskList.myLEDColorSetter.Value = 0;
            modulusInt = 0;
        }

    }

}