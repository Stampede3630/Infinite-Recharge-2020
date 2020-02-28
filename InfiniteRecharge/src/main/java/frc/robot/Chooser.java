/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;

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
    
    private static enum RobotState  {
        
        INTAKE, INITIATION_LINE_SHOT, SHORT_TRENCH, LONG_SHOT, NO_MANS_LAND
    }
    private RobotState currentRobotState = RobotState.INTAKE;
    private SendableChooser<Boolean> climbChooser;
    private boolean fieldRelative;

    private Chooser() {

        climbChooser = new SendableChooser<Boolean>();
        climbChooser.setDefaultOption("angles", true);
        climbChooser.addOption("climb", false);
        fieldRelative = true;
    }

    public void updateChooser() {
        if (climbChooser.getSelected()) {
            // run angle turn, hex line up

        } else {
            // run climb d-pad, climb angle //climb limelight angle
            Climber.getInstance().climberPeriodic();
        }
    }

    public void driveChooser() {
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
    

        /*
        else if (RobotMap.CONTROLLER.getPOV() == 45) { // BAD
            Drivetrain.getInstance().driveAtAngle(11, fieldRelative);
        }*/
    }

    public void robotStateChooser()
    {
        //read state and change RPM, kF, Limelight angle, pipeline, drive angle,
        switch(currentRobotState)
        {

        case INTAKE:
        RobotMap.StateChooser.LIMELIGHT_ANGLE = RobotMap.StateConstants.INTAKE_SERVO_POS;
        RobotMap.StateChooser.DRIVE_ANGLE = RobotMap.StateConstants.INTAKE_ANGLE;
        RobotMap.StateChooser.PIPELINE = RobotMap.StateConstants.INTAKE_PIPELINE;
        break;

        case INITIATION_LINE_SHOT:
        RobotMap.StateChooser.LIMELIGHT_ANGLE = RobotMap.StateConstants.INITIATION_LINE_SHOT_SERVO_POS;
        RobotMap.StateChooser.DRIVE_ANGLE = RobotMap.StateConstants.INITIATION_LINE_SHOT_ANGLE;
        RobotMap.StateChooser.PIPELINE = RobotMap.StateConstants.INITIATION_LINE_SHOT_PIPELINE;
        RobotMap.StateChooser.kF = RobotMap.StateConstants.INITIATION_LINE_SHOT_KF;
        RobotMap.StateChooser.RPM = RobotMap.StateConstants.INITIATION_LINE_SHOT_RPM;
        RobotMap.StateChooser.HOOD_ANGLE = RobotMap.StateConstants.INITIATION_LINE_SHOT_HOOD_ANGLE;
        break;

        case SHORT_TRENCH:
        RobotMap.StateChooser.LIMELIGHT_ANGLE = RobotMap.StateConstants.SHORT_TRENCH_SERVO_POS;
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


        



        }
    }

    public void chooserPeriodic()
    {
        /*
        //Hold button lineup chooser 
        if(button pressed)
        {
            hex line-up
        }
        else if( button pressed)
        {
            ball following
        }
        else
        {
            stop all automated lineup methods
        }

        //Limelight Angle Determination Co-driver override
        if(chooser gets shooter far)
        {
            shooter far angle set
        }
        else if (chooser gets shooter close)
        {
            shooter close angle set
        }
        else if(intake gets selected)
        {
            inake angle set
        }


    */
    }

}
