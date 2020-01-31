package frc.robot.PID;

import edu.wpi.first.wpilibj.PIDSource;
import edu.wpi.first.wpilibj.PIDSourceType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.DriverStation;
import frc.robot.Constants;

public class XpidSource implements PIDSource
{

    static double xDist, tx, tempTx, initTime;

    public XpidSource()
    {

    }

    public PIDSourceType getPIDSourceType()
    {
        return PIDSourceType.kDisplacement;
    }

    public void setPIDSourceType(PIDSourceType source)
    {

    }

    public double pidGet()
    {
        if(Constants.ts>=-80&&Constants.ts<=-10)
        {
            NetworkTableInstance.getDefault().getTable(Constants.limelight).getEntry("snapshot").setNumber(1);
            return tempTx;
        }
/*        else
        {
            tempTx = Constants.tx;
            return Constants.tx;
        }
        */


        else if(!Constants.ballManipulator)
        {
            tempTx = Constants.tx;
            double toReturn = -Math.tan(degreesToRadians(Constants.tx))*((Constants.h1-Constants.h2)/Math.tan(degreesToRadians(Constants.ty-15)));
            return toReturn;
        }
        else 
        {
            tempTx = Constants.tx;
            double toReturn = -Math.tan(degreesToRadians(Constants.tx))*((Constants.h3-Constants.h2)/Math.tan(degreesToRadians(Constants.ty+35)));
            return toReturn;
        }
        
    }


//PID calculations
//        double dist = Math.tan(degreesToRadians(tx))*((Constants.h2 - Constants.h1) / Math.tan(degreesToRadians(Constants.alphaYOne + ty)));
//        SmartDashboard.putNumber("xDistance", dist);

    public static double degreesToRadians(double theta) {
        double radians = theta * (Math.PI / 180);
        return radians;
    }
}

