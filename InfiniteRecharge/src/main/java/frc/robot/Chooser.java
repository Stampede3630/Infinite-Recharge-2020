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
    private SendableChooser<Boolean> climbChooser;
    private  Drivetrain m_swerve;
    boolean fieldRelative;

    private Climber climber;
    public Chooser(){

        climbChooser = new SendableChooser<Boolean>();
        climber = new Climber();
        climbChooser.setDefaultOption("angles", true);
        climbChooser.addOption("climb", false);
        m_swerve = Drivetrain.getInstance();
        fieldRelative = true;
    }

    public void updateChooser(){
        if(climbChooser.getSelected()){
            //run angle turn, hex line up

        }
        else{
            //run climb d-pad, climb angle
            climber.climberPeriodic();
        }
    }

    public void driveChooser()
    {
        if (RobotMap.CONTROLLER.getPOV() == 0) {
            m_swerve.driveAtAngle(0, fieldRelative);
        }
        else if (RobotMap.CONTROLLER.getPOV()==180) {
            m_swerve.driveAtAngle(180, fieldRelative);
        }
        else if (RobotMap.CONTROLLER.getPOV()==270) {
            m_swerve.driveAtAngle(270, fieldRelative);
        }
        else if (RobotMap.CONTROLLER.getPOV()==90) {
            m_swerve.driveAtAngle(90, fieldRelative);
        }
        else if (RobotMap.CONTROLLER.getPOV() == 45) //BAD
        {
            m_swerve.driveAtAngle(11, fieldRelative);
        }
        else {
            m_swerve.driveWithJoystick(fieldRelative);
        }
    }


}
