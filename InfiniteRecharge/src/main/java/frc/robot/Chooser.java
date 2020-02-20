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
            // run climb d-pad, climb angle
            Climber.getInstance().climberPeriodic();
        }
    }

    public void driveChooser() {
        if (RobotMap.CONTROLLER.getPOV() == 0) {
            Drivetrain.getInstance().driveAtAngle(0, fieldRelative);
        } else if (RobotMap.CONTROLLER.getPOV() == 180) {
            Drivetrain.getInstance().driveAtAngle(180, fieldRelative);
        } else if (RobotMap.CONTROLLER.getPOV() == 270) {
            Drivetrain.getInstance().driveAtAngle(270, fieldRelative);
        } else if (RobotMap.CONTROLLER.getPOV() == 90) {
            Drivetrain.getInstance().driveAtAngle(90, fieldRelative);
        } else if (RobotMap.CONTROLLER.getPOV() == 45) { // BAD
            Drivetrain.getInstance().driveAtAngle(11, fieldRelative);
        } else {
            Drivetrain.getInstance().driveWithJoystick(fieldRelative);
        }
    }

}
