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
            RobotMap.StateConstants.ALLOW_AUTOMATED_CONTROL = false;
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
//
    public void driveChooser() {
       
       if(RobotMap.CONTROLLER.getAButton())
       {
            RobotMap.StateConstants.ALLOW_AUTOMATED_CONTROL = false;
       }
       if(RobotMap.CONTROLLER.getBButton())
       {
        RobotMap.StateConstants.ALLOW_AUTOMATED_CONTROL = true;
       }
       if(currentRobotState != pastRobotState)
       {
        RobotMap.StateConstants.ALLOW_AUTOMATED_CONTROL = true;
        pastRobotState = currentRobotState;
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
    

    public void robotStateChooser()
    {
        //read state and change RPM, kF, Limelight angle, pipeline, drive angle,
        if(!DriverStation.getInstance().isAutonomous())
        {
        currentRobotState = stateChooser.getSelected();
        switch(currentRobotState)
        {

        case INTAKE:
        RobotMap.StateChooser.LIMELIGHT_ANGLE = RobotMap.StateConstants.INTAKE_SERVO_POS;
        RobotMap.StateChooser.DRIVE_ANGLE = RobotMap.StateConstants.INTAKE_ANGLE;
        RobotMap.StateChooser.PIPELINE = RobotMap.StateConstants.INTAKE_PIPELINE;
        break;

        case INITIATION_LINE_SHOT:
        RobotMap.StateChooser.LIMELIGHT_ANGLE = RobotMap.StateConstants.INITIATION_LINE_SHOT_SERVO_ANGLE;
        RobotMap.StateChooser.DRIVE_ANGLE = RobotMap.StateConstants.INITIATION_LINE_SHOT_ANGLE;
        RobotMap.StateChooser.PIPELINE = RobotMap.StateConstants.INITIATION_LINE_SHOT_PIPELINE;
        RobotMap.StateChooser.kF = RobotMap.StateConstants.INITIATION_LINE_SHOT_KF;
        RobotMap.StateChooser.RPM = RobotMap.StateConstants.INITIATION_LINE_SHOT_RPM;
        RobotMap.StateChooser.HOOD_ANGLE = RobotMap.StateConstants.INITIATION_LINE_SHOT_HOOD_ANGLE;
        break;

        case SHORT_TRENCH:
        RobotMap.StateChooser.LIMELIGHT_ANGLE = RobotMap.StateConstants.SHORT_TRENCH_SERVO_ANGLE;
        RobotMap.StateChooser.DRIVE_ANGLE = RobotMap.StateConstants.SHORT_TRENCH_ANGLE;
        RobotMap.StateChooser.PIPELINE = RobotMap.StateConstants.SHORT_TRENCH_PIPELINE;
        RobotMap.StateChooser.kF = RobotMap.StateConstants.SHORT_TRENCH_KF;
        RobotMap.StateChooser.RPM = RobotMap.StateConstants.SHORT_TRENCH_RPM;
        RobotMap.StateChooser.HOOD_ANGLE = RobotMap.StateConstants.SHORT_TRENCH_HOOD_ANGLE;
        break;

        case LONG_SHOT:
        RobotMap.StateChooser.LIMELIGHT_ANGLE = RobotMap.StateConstants.LONG_SHOT_SERVO_POS;
        RobotMap.StateChooser.DRIVE_ANGLE = RobotMap.StateConstants.LONG_SHOT_ANGLE;
        RobotMap.StateChooser.PIPELINE = RobotMap.StateConstants.LONG_SHOT_PIPELINE;
        RobotMap.StateChooser.kF = RobotMap.StateConstants.LONG_SHOT_KF;
        RobotMap.StateChooser.RPM = RobotMap.StateConstants.LONG_SHOT_RPM;
        RobotMap.StateChooser.HOOD_ANGLE = RobotMap.StateConstants.LONG_SHOT_HOOD_ANGLE;
        break;

        case NO_MANS_LAND:
        RobotMap.StateChooser.LIMELIGHT_ANGLE = RobotMap.StateConstants.NO_MANS_LAND_SERVO_POS;
        RobotMap.StateChooser.DRIVE_ANGLE = RobotMap.StateConstants.NO_MANS_LAND_ANGLE;
        RobotMap.StateChooser.PIPELINE = RobotMap.StateConstants.NO_MANS_LAND_PIPELINE;
        RobotMap.StateChooser.kF = RobotMap.StateConstants.NO_MANS_LAND_KF;
        RobotMap.StateChooser.RPM = RobotMap.StateConstants.NO_MANS_LAND_RPM;
        RobotMap.StateChooser.HOOD_ANGLE = RobotMap.StateConstants.NO_MANS_LAND_HOOD_ANGLE;
        break;

        case RIGHT_CLIMB:
        RobotMap.StateChooser.LIMELIGHT_ANGLE = RobotMap.StateConstants.CLIMBER_SERVO_POS;
        RobotMap.StateChooser.DRIVE_ANGLE = RobotMap.StateConstants.RIGHT_CLIMBER_ANGLE;
        RobotMap.StateChooser.PIPELINE = RobotMap.StateConstants.CLIMBER_PIPELINE;
        break;

        case LEFT_CLIMB:
        RobotMap.StateChooser.LIMELIGHT_ANGLE = RobotMap.StateConstants.CLIMBER_SERVO_POS;
        RobotMap.StateChooser.DRIVE_ANGLE = RobotMap.StateConstants.LEFT_CLIMBER_ANGLE;
        RobotMap.StateChooser.PIPELINE = RobotMap.StateConstants.CLIMBER_PIPELINE;
        
        }
    }

       

    }

    public void autoChooser(RobotState currentRobotState)
    {
        switch(currentRobotState)
        {

        case INTAKE:
        RobotMap.StateChooser.LIMELIGHT_ANGLE = RobotMap.StateConstants.INTAKE_SERVO_POS;
        RobotMap.StateChooser.DRIVE_ANGLE = RobotMap.StateConstants.INTAKE_ANGLE;
        RobotMap.StateChooser.PIPELINE = RobotMap.StateConstants.INTAKE_PIPELINE;
        break;

        case INITIATION_LINE_SHOT:
        RobotMap.StateChooser.LIMELIGHT_ANGLE = RobotMap.StateConstants.INITIATION_LINE_SHOT_SERVO_ANGLE;
        RobotMap.StateChooser.DRIVE_ANGLE = RobotMap.StateConstants.INITIATION_LINE_SHOT_ANGLE;
        RobotMap.StateChooser.PIPELINE = RobotMap.StateConstants.INITIATION_LINE_SHOT_PIPELINE;
        RobotMap.StateChooser.kF = RobotMap.StateConstants.INITIATION_LINE_SHOT_KF;
        RobotMap.StateChooser.RPM = RobotMap.StateConstants.INITIATION_LINE_SHOT_RPM;
        RobotMap.StateChooser.HOOD_ANGLE = RobotMap.StateConstants.INITIATION_LINE_SHOT_HOOD_ANGLE;
        break;

        case SHORT_TRENCH:
        RobotMap.StateChooser.LIMELIGHT_ANGLE = RobotMap.StateConstants.SHORT_TRENCH_SERVO_ANGLE;
        RobotMap.StateChooser.DRIVE_ANGLE = RobotMap.StateConstants.SHORT_TRENCH_ANGLE;
        RobotMap.StateChooser.PIPELINE = RobotMap.StateConstants.SHORT_TRENCH_PIPELINE;
        RobotMap.StateChooser.kF = RobotMap.StateConstants.SHORT_TRENCH_KF;
        RobotMap.StateChooser.RPM = RobotMap.StateConstants.SHORT_TRENCH_RPM;
        RobotMap.StateChooser.HOOD_ANGLE = RobotMap.StateConstants.SHORT_TRENCH_HOOD_ANGLE;
        break;

        case LONG_SHOT:
        RobotMap.StateChooser.LIMELIGHT_ANGLE = RobotMap.StateConstants.LONG_SHOT_SERVO_POS;
        RobotMap.StateChooser.DRIVE_ANGLE = RobotMap.StateConstants.LONG_SHOT_ANGLE;
        RobotMap.StateChooser.PIPELINE = RobotMap.StateConstants.LONG_SHOT_PIPELINE;
        RobotMap.StateChooser.kF = RobotMap.StateConstants.LONG_SHOT_KF;
        RobotMap.StateChooser.RPM = RobotMap.StateConstants.LONG_SHOT_RPM;
        RobotMap.StateChooser.HOOD_ANGLE = RobotMap.StateConstants.LONG_SHOT_HOOD_ANGLE;
        break;

        case NO_MANS_LAND:
        RobotMap.StateChooser.LIMELIGHT_ANGLE = RobotMap.StateConstants.NO_MANS_LAND_SERVO_POS;
        RobotMap.StateChooser.DRIVE_ANGLE = RobotMap.StateConstants.NO_MANS_LAND_ANGLE;
        RobotMap.StateChooser.PIPELINE = RobotMap.StateConstants.NO_MANS_LAND_PIPELINE;
        RobotMap.StateChooser.kF = RobotMap.StateConstants.NO_MANS_LAND_KF;
        RobotMap.StateChooser.RPM = RobotMap.StateConstants.NO_MANS_LAND_RPM;
        RobotMap.StateChooser.HOOD_ANGLE = RobotMap.StateConstants.NO_MANS_LAND_HOOD_ANGLE;
        break;

        case RIGHT_CLIMB:
        RobotMap.StateChooser.LIMELIGHT_ANGLE = RobotMap.StateConstants.CLIMBER_SERVO_POS;
        RobotMap.StateChooser.DRIVE_ANGLE = RobotMap.StateConstants.RIGHT_CLIMBER_ANGLE;
        RobotMap.StateChooser.PIPELINE = RobotMap.StateConstants.CLIMBER_PIPELINE;
        break;

        case LEFT_CLIMB:
        RobotMap.StateChooser.LIMELIGHT_ANGLE = RobotMap.StateConstants.CLIMBER_SERVO_POS;
        RobotMap.StateChooser.DRIVE_ANGLE = RobotMap.StateConstants.LEFT_CLIMBER_ANGLE;
        RobotMap.StateChooser.PIPELINE = RobotMap.StateConstants.CLIMBER_PIPELINE;
        
        }

    }

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

    
    public void intakeChooser()
    {
        if(SmartDashboard.getBoolean("Intake Baby?", true))
        {
            IntakeIndex.getInstance().intakeBaby();
        }
        else
        {
            IntakeIndex.getInstance().index();
        }
    }
    public void chooserPeriodic()
    {
        resetYaw();
        robotStateChooser();
        driveChooser();
        fieldRelative();
    }

}
