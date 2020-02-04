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
  

    public DriveTrain() {
        mD = new MecanumDrive(RobotMap.talonFL, RobotMap.talonBL, RobotMap.talonFR, RobotMap.talonBR);
    }

    public void teleOpDrive() {
        mD.driveCartesian(RobotMap.controller.getX(Hand.kLeft) * 0.5, RobotMap.controller.getY(Hand.kLeft) * -0.5, RobotMap.controller.getX(Hand.kRight));
}} 
 //new change 
 //new change 
 //new change 