package frc.robot;

public class Manipulator
{
    ManipulatorMode manipulatorMode;
    RobotMap robotMap;
    public double robotTime;

    public Manipulator()
    {
        robotMap = RobotMap.getRobotMap();
    }

    public void manipulatorPeriodic()
    {
        if(!Constants.ballFollowerExecuting)
        {
            if(robotMap.bumperL.get())
            {
                manipulatorMode.disengage();//left bumper
            }
            else if(robotMap.bumperR.get())
            {
                manipulatorMode.engage(); //right bumper
            }
            else if(robotMap.getTrigger()<0)
            {
                manipulatorMode.deploy(Constants.toRocket); //left trigger
            }
            else if(robotMap.getTrigger()>0)
            {
                manipulatorMode.intake(); //right trigger
            }
            /*m
            else if(Constants.autoDeploy)
            {
                manipulatorMode.deployAuto(robotTime);
            }
            */
            else 
            {
                manipulatorMode.endAll();
            }
        }
    }
}