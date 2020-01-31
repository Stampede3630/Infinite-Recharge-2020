package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.Timer;
import com.ctre.phoenix.CANifier.LEDChannel;

public class Diagnostics
{

    public RobotMap robotMap;

    double sonicDistance;
    boolean hatchIn;

    public Diagnostics()
    {
        this.robotMap = RobotMap.getRobotMap();
    }

    public void toSmartDashboard()
    {
        SmartDashboard.putNumber("Pressure", getPSI());
        
        SmartDashboard.putNumber("ahrs", robotMap.ahrs.getYaw());
       
        SmartDashboard.putBoolean("ball in high", robotMap.ballButton.get());

        SmartDashboard.putBoolean("hatch in R", robotMap.dumbHatchButton.get());
        
        SmartDashboard.putBoolean("hatch in L", robotMap.hatchButton.get());

        SmartDashboard.putBoolean("ball in low", robotMap.lowBallButton.get());

        SmartDashboard.putNumber("distance in x", -Math.tan(degreesToRadians(Constants.tx))*((Constants.h1-Constants.h2)/Math.tan(degreesToRadians(Constants.ty-15))));


//        SmartDashboard.putNumber("calculations", Math.tan(degreesToRadians(Constants.tx))*((Constants.h1-Constants.h2)/Math.tan(Constants.ty-15)));
        
    }

    public static double degreesToRadians(double theta) {
        double radians = theta * (Math.PI / 180);
        return radians;
    }

    public double getPSI()
    {
        double sensorV = robotMap.pressureLevel.getVoltage();
        double psi = 250*(sensorV / 5) - 25;
        psi = Math.round(psi);
        return psi;
    }

    public void limelightValues()
    {
        Constants.tv = NetworkTableInstance.getDefault().getTable(Constants.limelight).getEntry("tv").getDouble(0);
        Constants.ty = NetworkTableInstance.getDefault().getTable(Constants.limelight).getEntry("ty").getDouble(0);
        Constants.ta = NetworkTableInstance.getDefault().getTable(Constants.limelight).getEntry("ta").getDouble(0);
        Constants.tx = NetworkTableInstance.getDefault().getTable(Constants.limelight).getEntry("tx").getDouble(0);
    }

    public void getForwardMode()
    {
        Constants.forwardFromWidget = Robot.hatchBallSelected.getString("");
    }

    public void SolenoidReset() 
    {
        robotMap.solenoidBack.set(Value.kReverse);
        robotMap.solenoidFront.set(Value.kReverse);
    }

    public void periodicVisionChange()
    {
        NetworkTableInstance.getDefault().getTable(Constants.limelight).getEntry("pipeline").setNumber(Constants.pipelineNumber);   
    }

    public void ultrasonicSensorReading()
    {
        sonicDistance = robotMap.ultrasonicSensor.getRangeInches();

        if (sonicDistance < 10.5)
        {
            hatchIn = true;
        }
        else 
        {
            hatchIn = false;
        }

       // SmartDashboard.putBoolean("Hatch In", hatchIn);
        SmartDashboard.putNumber("inches from vex", sonicDistance);
        
    }

    public void resetGyro ()
    {
        if(robotMap.startB.get())
        {
            robotMap.ahrs.reset();
        }
    }
    

    /*public void flashLights()
    {
        if(!robotMap.hatchButton.get())
        {
            robotMap.canifier.setLEDOutput(0.3, LEDChannel.LEDChannelA);
            robotMap.canifier.setLEDOutput(0.3, LEDChannel.LEDChannelB);
        }
        else
        {
            robotMap.canifier.setLEDOutput(0, LEDChannel.LEDChannelA);
            robotMap.canifier.setLEDOutput(0, LEDChannel.LEDChannelB);
            robotMap.canifier.setLEDOutput(0, LEDChannel.LEDChannelC);
        }
    }*/
}