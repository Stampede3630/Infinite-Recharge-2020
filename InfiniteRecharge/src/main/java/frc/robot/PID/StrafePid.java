package frc.robot.PID;

import edu.wpi.first.wpilibj.PIDController;
import edu.wpi.first.wpilibj.PIDOutput;
import edu.wpi.first.wpilibj.PIDBase.Tolerance;
import edu.wpi.first.wpilibj.livewindow.LiveWindow;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

public class StrafePid implements PIDOutput 
{

    public PIDController strafeController;
    private XpidSource source;
    private double strafeOutput;
    private MyCustomTolerance newTolerance;

    public StrafePid()
    {

    }

    public void strafePidSetup()
    {
        newTolerance = new MyCustomTolerance();
        source = new XpidSource();
        strafeController = new PIDController(Constants.strafeKP, Constants.strafeKI, Constants.strafeKD, source, this);
        strafeController.setOutputRange(-Constants.strafeOutput, Constants.strafeOutput);
        strafeController.setTolerance(newTolerance);
        strafeController.disable();
        LiveWindow.add(strafeController);
    }

    public double getStrafeOutput()
    {
        SmartDashboard.putNumber("x PID error", source.pidGet());
        SmartDashboard.putNumber("x PID output", strafeOutput);
        return strafeOutput;
    }

    public void pidWrite(double output)
    {
        strafeOutput = output;
    }

    public class MyCustomTolerance implements Tolerance {
        private final double m_posTolerance;
        private final double m_velocityLimit;

        MyCustomTolerance() 
        {
            m_posTolerance = Constants.strafeTolerance;
            m_velocityLimit = Constants.pidLowSpeed; 
        }

        @Override
        public boolean onTarget() 
        {
            return Math.abs(strafeController.getError()) < m_posTolerance && Math.abs(strafeController.get()) < m_velocityLimit;
        }
    }
}