/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import frc.robot.RobotMap;

/**
 * Add your docs here.
 */
public class DriveTrain {
    MecanumDrive mD;
    RobotMap robotMap;

    public DriveTrain() {
        mD = new MecanumDrive(robotMap.talonFL, robotMap.talonBL, robotMap.talonFR, robotMap.talonBR);
    }

    public void teleOpDrive() {
        mD.driveCartesian(robotMap.controller.getY(Hand.kRight) * 0.5, robotMap.controller.getX(Hand.kLeft) * 0.5, 0);
    }
    
}
