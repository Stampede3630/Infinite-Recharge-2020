package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Choosers
{

    RobotMap robotMap;
    Robot robot;
    boolean toRocket, ballManipulator;
    DriveTrain driveTrain;
    Manipulator manipulator;
    String currentManipulator = "";
    String comparePath = "";
    PathChooser pathChooser;
    Boolean bButtonPressed;

    public Choosers(DriveTrain driveTrain, Manipulator manipulator)
    {
        robotMap = RobotMap.getRobotMap();


        this.driveTrain = driveTrain;
        driveTrain.driveMode = new ManualDrive(robotMap, driveTrain);
        this.manipulator = manipulator;
        manipulator.manipulatorMode = new Hatch(manipulator);

        toRocket = false;
        ballManipulator = false;

    }

    public double reverseAngle(double angle) {
        double outputAngle = angle;
        if (Constants.ballManipulator)
        {
            if(outputAngle<0)
            {
                outputAngle = angle + 180;
            }
            else
            {
                outputAngle = angle - 180;
            }
        }
        return outputAngle;
    }

    public void chooserAngle(double angle) {
        if (driveTrain.driveMode.getAutoRotate()) {

            driveTrain.turnPID.turnController.setSetpoint(angle);
            driveTrain.turnSetpoint = angle;
            driveTrain.turnPID.turnController.enable();
        }

        else {
            driveTrain.turnPID.turnController.disable();
        }
    }

    public void setDriveMode()
    {
/*        if(driveTrain.forwardPID.)
        {
            driveTrain.driveMode = new ManualDrive(robotMap, driveTrain);
            Constants.lostTarget = false;
        }
*/
        if(robotMap.buttonA.get())
        {
            driveTrain.driveMode = new ManualDrive(robotMap, driveTrain);
        }
        if(robotMap.buttonB.get())
        {
            driveTrain.driveMode = new VisionDrive(robotMap, driveTrain);
            
        }
    }

    public void updatePath()
    {
         if (!(robot.pathSelected.toString().equals(comparePath)))
        {
            //pathChooser.stringToPath(robot.pathSelected.toString());
            comparePath = robot.pathSelected.toString();
            NetworkTableInstance.getDefault().getTable(Constants.limelight).getEntry("pipeline").setNumber(Constants.pipelineNumber);
        }
    }

    public void setManipulatorMode()
    {
//        if(!currentManipulator.equals(Robot.manipulatorChooser.getSelected()))
        if(!currentManipulator.equals(Constants.forwardFromWidget))
        {
/*            if(!currentManipulator.equals("cargo"))
            {
               
                //System.out.println("Manipulator should have disengaged");
            }
*/
            //currentManipulator = Robot.manipulatorChooser.getSelected().toString();
            currentManipulator = Constants.forwardFromWidget;

//            if(currentManipulator.equals("Ball"))
            if(currentManipulator.equals("cargo"))
            {
                NetworkTableInstance.getDefault().getTable(Constants.ballLimelight).getEntry("pipeline").setNumber(Constants.closestIntake);
                robotMap.hatchExtend.set(Value.kForward);
                manipulator.manipulatorMode = new Ball(manipulator);
                Constants.limelight = Constants.ballLimelight;
                Constants.ballManipulator = true;
                NetworkTableInstance.getDefault().getTable(Constants.hatchLimelight).getEntry("pipeline").setNumber(Constants.defaultPL);

            }
            else 
            {
                NetworkTableInstance.getDefault().getTable(Constants.hatchLimelight).getEntry("pipeline").setNumber(Constants.hatchDriving);
                manipulator.manipulatorMode = new Hatch(manipulator);
                Constants.limelight = Constants.hatchLimelight;
                Constants.ballManipulator = false;
                NetworkTableInstance.getDefault().getTable(Constants.ballLimelight).getEntry("pipeline").setNumber(Constants.defaultPL);
            }

        }
    }

    public void letterButtons()
    {
        if(robotMap.buttonX.get())
        {
            NetworkTableInstance.getDefault().getTable(Constants.ballLimelight).getEntry("pipeline").setNumber(Constants.closestIntake);
            NetworkTableInstance.getDefault().getTable(Constants.hatchLimelight).getEntry("pipeline").setNumber(Constants.hatchDriving);
            
        }
        if(robotMap.buttonB.get())
        {
            NetworkTableInstance.getDefault().getTable(Constants.limelight).getEntry("pipeline").setNumber(Constants.pipelineNumber);
            
        }
        if(robotMap.buttonA.get())
        {
            NetworkTableInstance.getDefault().getTable(Constants.ballLimelight).getEntry("pipeline").setNumber(Constants.closestIntake);
            NetworkTableInstance.getDefault().getTable(Constants.hatchLimelight).getEntry("pipeline").setNumber(Constants.hatchDriving);
            
        }
        if(robotMap.buttonY.get())
        {
            NetworkTableInstance.getDefault().getTable(Constants.limelight).getEntry("pipeline").setNumber(Constants.pipelineNumber);
            
        }
    }

}

/*

    public void automatedTurnToAngle ()
    {
        if(!toRocket)
        {
            if((robotMap.controller.getPOV()) == 0) 
            { 
                driveTrain.turnPID.zController.setSetpoint(0);
                driveTrain.turnPID.zController.enable();
                driveTrain.turnSetpoint = 0;
            }
            else if((robotMap.controller.getPOV()) == 90) 
            { 
                driveTrain.turnPID.zController.setSetpoint(90);
                driveTrain.turnPID.zController.enable();
                driveTrain.turnSetpoint = 90;
            }
            else if ((robotMap.controller.getPOV()) == 180) 
            { 
                driveTrain.turnPID.zController.setSetpoint(180);
                driveTrain.turnPID.zController.enable();
                driveTrain.turnSetpoint = 180;
            }
            else if((robotMap.controller.getPOV()) == 270) 
            { 
                driveTrain.turnPID.zController.setSetpoint(-90);
                driveTrain.turnPID.zController.enable();
                driveTrain.turnSetpoint = -90;
            }
        }
        else if(toRocket)
        {
            if(robotMap.controller.getPOV() == 45) 
            { 
                driveTrain.turnPID.zController.setSetpoint(29);
                driveTrain.turnPID.zController.enable();
                driveTrain.turnSetpoint = 29;
            }
            else if(robotMap.controller.getPOV() == 315) 
            { 
                driveTrain.turnPID.zController.setSetpoint(-29);
                driveTrain.turnPID.zController.enable();
                driveTrain.turnSetpoint = -29;
            }
            else if(robotMap.controller.getPOV() == 135)
            {
                driveTrain.turnPID.zController.setSetpoint(151);
                driveTrain.turnPID.zController.enable();
                driveTrain.turnSetpoint = 151;
            }
            else if(robotMap.controller.getPOV() == 225) 
            {
                driveTrain.turnPID.zController.setSetpoint(-151);
                driveTrain.turnPID.zController.enable();
                driveTrain.turnSetpoint = -151;
            }
        }
    }
*/