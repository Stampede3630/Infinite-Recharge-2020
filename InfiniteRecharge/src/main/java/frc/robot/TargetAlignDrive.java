/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import frc.robot.Limelight.Target.TargetType;

/**
 * Add your docs here.
 */
public class TargetAlignDrive {

	public static void stop() {
		Drivetrain.stop();
	}

	public static boolean drive()
	{
		Limelight.Target.trackTarget(TargetType.UpperTarget);

		if (Limelight.Target.isValid())
		{
			return align();
		}
		else
		{
			searchTarget();
			return false;
		}
	}

	private static boolean align()
	{
		if (!Limelight.Target.isValid()) return false; // Failsafe

		double angleVel = RobotMap.TrackingPIDMap.TURN.calculate(Limelight.Target.getAngle(), 0);

		Drivetrain.drive(0, 0, angleVel, false); // TODO: Clamp the angle velocity?

		return Limelight.Target.getAngle() < RobotMap.TargetAlignMap.ANGLE_THRESHOLD;
	}

	private static void searchTarget()
	{

	}
}
