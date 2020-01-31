package frc.robot;


public class ManualDrive implements DriveMode {

    RobotMap robotMap;
    DriveTrain driveTrain;
    double xSpeed;
    double ySpeed;
    double zRotation;
    boolean autoRotateEnable;
    
    public ManualDrive(RobotMap robotMap, DriveTrain driveTrain)
    {
        this.robotMap = robotMap;
        this.driveTrain=driveTrain;

//        NetworkTableInstance.getDefault().getTable("limelight-one").getEntry("camMode").setNumber(1);
//        NetworkTableInstance.getDefault().getTable("limelight-two").getEntry("camMode").setNumber(1);
//        driveTrain.turnPID.zController.enable();
    }

    public boolean getAutoRotate() {
        return autoRotateEnable;
    }

    public void driveRobot()
    {
        Constants.ballFollowerExecuting = false;
//        if (Robot.manipulatorChooser.getSelected().equals("Ball")) //grab the value from Constants
        if(Constants.forwardFromWidget.equals("cargo")) //just testing that orientation thing
        {
            xSpeed = -robotMap.getLeftX();
            ySpeed = -robotMap.getLeftY();
        }
        else
        {
            xSpeed = robotMap.getLeftX();
            ySpeed = robotMap.getLeftY();
        }

        if (robotMap.buttonA.get()) {
            autoRotateEnable = true;
            driveTrain.turnPID.turnController.enable();
        }

        if (robotMap.buttonA.get()) {
            autoRotateEnable = true;
        }

        if (robotMap.buttonA.get()) {
            autoRotateEnable = true;
        }

        if(Math.abs(robotMap.getRightX())>0.2)
        {
            driveTrain.turnPID.turnController.disable();
            zRotation = robotMap.getRightX();
            autoRotateEnable = false;
        }
        else
        {
            driveTrain.turnPID.turnController.setSetpoint(Constants.robotAngle);
            zRotation = driveTrain.turnPID.getTurnOutput();
        }

        if (robotMap.leftStickB.get()||robotMap.rightStickB.get())
        {
            robotMap.drive.driveCartesian(xSpeed, ySpeed, zRotation);
        }
        else
        {
            robotMap.drive.driveCartesian(xSpeed * Constants.normalSpeed, ySpeed * Constants.normalSpeed, zRotation * Constants.normalSpeed);
        }
    }

}