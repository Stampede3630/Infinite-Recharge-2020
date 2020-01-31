package frc.robot.PID;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import frc.robot.Constants;
import edu.wpi.first.wpilibj.smartdashboard.*;
import edu.wpi.first.wpilibj.PIDBase.Tolerance;

public class ForwardPid implements PIDOutput
{

    public PIDController forwardController;
    private YpidSource source;
    private double frontOutput;
    private MyCustomTolerance newTolerance;

    public ForwardPid()
    {

    }

    public void forwardPidSetup()
    {
        newTolerance = new MyCustomTolerance();
        source = new YpidSource();
        forwardController = new PIDController(Constants.forwardKP, Constants.forwardKI, Constants.forwardKD, source, this);
        forwardController.setOutputRange(-Constants.forwardOutput, Constants.forwardOutput);
        forwardController.setTolerance(newTolerance);
        forwardController.disable();
        LiveWindow.add(forwardController);
    }

    public double getForwardOutput()
    {
        SmartDashboard.putNumber("y PID error", forwardController.getError());
        SmartDashboard.putNumber("y PID output", frontOutput);
        return frontOutput;
    }

    public void pidWrite(double output)
    {
        frontOutput = output;
    }

    public class MyCustomTolerance implements Tolerance 
    {
        private final double m_posTolerance;
        private final double m_velocityLimit;

        MyCustomTolerance() 
        {
            m_posTolerance = Constants.forwardTolerance;
            m_velocityLimit = Constants.pidLowSpeed; 
        }

        @Override
        public boolean onTarget() 
        {
            return Math.abs(forwardController.getError()) < m_posTolerance && Math.abs(forwardController.get()) < m_velocityLimit;
        }
    }
}