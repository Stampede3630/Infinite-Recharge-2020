/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class DriveTrain {
	private static final SpeedControllerGroup leftMotors = new SpeedControllerGroup(RobotMap.TALON_FL,
			RobotMap.TALON_BL);
	private static final SpeedControllerGroup rightMotors = new SpeedControllerGroup(RobotMap.TALON_FR,
			RobotMap.TALON_BR);
	private static final DifferentialDrive differentialDrive = new DifferentialDrive(leftMotors, rightMotors);

	public static void drive(Joystick joystick)
	{
		double speed = MathHelper.deadzone(-joystick.getY(), 0.05);
		double rotation = MathHelper.clampUnit(MathHelper.deadzone(joystick.getX(), 0.05) + MathHelper.deadzone(joystick.getZ(), 0.05));

		drive(speed, rotation);
	}

	public static void drive(double xSpeed, double zRotation)
	{

		differentialDrive.arcadeDrive(xSpeed * 0.5, zRotation * 0.7, true);
	}

	public static void stop()
	{
		drive(0, 0);
	}
}