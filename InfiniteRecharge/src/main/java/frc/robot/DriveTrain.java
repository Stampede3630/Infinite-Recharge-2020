/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class DriveTrain {
	private static final SpeedControllerGroup leftMotors = new SpeedControllerGroup(RobotMap.TALON_FL,
			RobotMap.TALON_BL);
	private static final SpeedControllerGroup rightMotors = new SpeedControllerGroup(RobotMap.TALON_FR,
			RobotMap.TALON_BR);
	private static final DifferentialDrive differentialDrive = new DifferentialDrive(leftMotors, rightMotors);

	public static void drive()
	{
		double speed = RobotMap.controller.getY(Hand.kLeft);
		double rotation = RobotMap.controller.getX(Hand.kLeft);

		if(Math.abs(RobotMap.controller.getY(Hand.kLeft))<0.1){
			speed = 0;
		}
		if(Math.abs(RobotMap.controller.getX(Hand.kLeft))<0.1){
			rotation = 0;
		}
		drive(speed, rotation);
	}

	public static void drive(double xSpeed, double zRotation)
	{

		differentialDrive.arcadeDrive(xSpeed, zRotation, true);
	}

	public static void stop()
	{
		drive(0, 0);
	}
}