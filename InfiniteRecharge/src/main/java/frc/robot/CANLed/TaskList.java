package frc.robot.CANLed;

import com.ctre.phoenix.ILoopable;

public class TaskList
{

    //public static RainbowMode operationRainbowMode = new RainbowMode();
    public static LEDColorSetter myLEDColorSetter = new LEDColorSetter();
    

    public static LEDMainLoop myMainLoop = new LEDMainLoop();


    public static ILoopable[] FullList = {myLEDColorSetter, myMainLoop};
}