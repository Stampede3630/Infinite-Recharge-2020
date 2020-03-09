/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import java.util.Map;
import java.util.function.BooleanSupplier;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class Chooser {

    private static Chooser instance;

    static {
        instance = new Chooser();
    }

    public static Chooser getInstance() {
        return instance;
    }

    public static enum RobotState {

        INTAKE, INITIATION_LINE_SHOT, SHORT_TRENCH, LONG_SHOT, NO_MANS_LAND, RIGHT_CLIMB, LEFT_CLIMB
    }

    private RobotState currentRobotState = RobotState.INTAKE;
    private RobotState pastRobotState = currentRobotState;
    private SendableChooser<RobotState> stateChooser;
    private boolean fieldRelative;
    private BooleanSupplier resetGyro;
    private boolean resetGyroBoolean;
    //private ShuffleboardTab gameDay;


    

    private Chooser() {
        stateChooser = new SendableChooser<RobotState>();
        SmartDashboard.putBoolean("Reset Gyro", false);
        SmartDashboard.putBoolean("Field Relative", true);
        SmartDashboard.putData(stateChooser);
        resetGyroBoolean = false;
        stateChooser.setDefaultOption("Initiation Line Shot", RobotState.INITIATION_LINE_SHOT);
        stateChooser.addOption("Intake", RobotState.INTAKE);
        stateChooser.addOption("Short Trench", RobotState.SHORT_TRENCH);
        stateChooser.addOption("Long Shot", RobotState.LONG_SHOT);
        stateChooser.addOption("No Mans Land", RobotState.NO_MANS_LAND);
        stateChooser.addOption("Right Climb", RobotState.RIGHT_CLIMB);
        stateChooser.addOption("Left Climb", RobotState.LEFT_CLIMB);
        //Shuffleboard.getTab("gameDay").add("Robot State", stateChooser);
      // Shuffleboard.getTab("gameDay").addBoolean("Reset Gyro", resetGyro).withWidget("Boolean Box")
       // .withProperties(Map.of("colorWhenTrue", "green", "colorWhenFalse", "maroon"));

    }

    public void resetGyro() {
       
        if(resetGyro.getAsBoolean())
        {
        
            RobotMap.SensorMap.GYRO.zeroYaw();
            resetGyroBoolean = false;
        }
    }

    public void resetYaw()
    {
       
        if(SmartDashboard.getBoolean("Reset Gyro", false))
        {
            RobotMap.SensorMap.GYRO.zeroYaw();
            SmartDashboard.putBoolean("Reset Gyro", false);
        }
    }


       //Initiation line/no mans land no angle?
       /*
        if (RobotMap.CONTROLLER.getPOV() == 0) {
            Drivetrain.getInstance().driveAtAngle(0, fieldRelative);
        } else if (RobotMap.CONTROLLER.getPOV() == 180) {
            Drivetrain.getInstance().driveAtAngle(180, fieldRelative);
        } else if (RobotMap.CONTROLLER.getPOV() == 270) {
            Drivetrain.getInstance().driveAtAngle(-90, fieldRelative);
        } else if (RobotMap.CONTROLLER.getPOV() == 90) {
            Drivetrain.getInstance().driveAtAngle(90, fieldRelative);
        }  else {
            Drivetrain.getInstance().driveWithJoystick(fieldRelative);
        }
        */

        /*
        else if (RobotMap.CONTROLLER.getPOV() == 45) { // BAD
            Drivetrain.getInstance().driveAtAngle(11, fieldRelative);
        }*/
    

   
    public void fieldRelative()
    {
        if(SmartDashboard.getBoolean("Field Relative", true))
        {
            RobotMap.StateChooser.FIELD_RELATIVE = true;
        }
        else
        {  
            RobotMap.StateChooser.FIELD_RELATIVE = false;
        }
    }

    

    public void chooserPeriodic()
    {
        resetYaw();
        fieldRelative();
    }

}
