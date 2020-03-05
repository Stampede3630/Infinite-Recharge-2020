/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Limelight.Target.TargetType;

/**
 * Add your docs here.
 */
public class TargetAlignDrive {

	private static TargetAlignDrive instance;

	//private RumbleSequence targetLockRumble = new RumbleSequence("RR--RR", 8);

	private double searchDirection;

	static {
		instance = new TargetAlignDrive();
	}

	public static TargetAlignDrive getInstance() {
		return instance;
	}

	private TargetAlignDrive() {

	}

	public void stop() {
		Drivetrain.getInstance().stop();
	}

	public double align() {
		if (!Limelight.Target.isValid())
			return 0; // Failsafe

		double angleVel = RobotMap.TrackingPIDMap.TURN.calculate(-Limelight.Target.getAngle(), 0)
				* RobotMap.DriveMap.MAX_ANGULAR_SPEED;
		SmartDashboard.putNumber("angleVel", angleVel);

		//Drivetrain.getInstance().drive(0, 0, angleVel, true); // TODO: Clamp the angle velocity?

		return angleVel;
	}

	private void searchTarget() {
		if (searchDirection == 0) {
			searchDirection = Math.signum(((RobotMap.SensorMap.GYRO.getAngle() % 360) + 180) % 360 - 180)
					* RobotMap.DriveMap.MAX_ANGULAR_SPEED;
		}
		Drivetrain.getInstance().drive(0, 0, -searchDirection / 8, true);
	}

	public void periodic() {
		SmartDashboard.putNumber("Angle to 0", ((RobotMap.SensorMap.GYRO.getAngle() % 360) + 180) % 360 - 180);
		SmartDashboard.putNumber("searchDirection", searchDirection);
		Limelight.Target.trackTarget(TargetType.UpperTarget);

		if (RobotMap.CONTROLLER.getBumper(Hand.kLeft)) {
			Limelight.setPipeline(RobotMap.Pipelines.TARGET_LINEUP_ZOOM);
			if (Limelight.Target.isValid()) {
				SmartDashboard.putNumber("Delta Angle", Math.abs(align()));
			} else {
				searchTarget();
			}
		} else {
			searchDirection = 0;
		}
	}
}