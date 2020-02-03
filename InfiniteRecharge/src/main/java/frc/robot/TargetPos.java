package frc.robot;

/*----------------------------------------------------------------------------*/

/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

/**
 * Add your docs here.
 */
public class TargetPos {
	public static final double CAM_HEIGHT = 33;
	public static final double CAM_ANGLE = -30;
	public static final double CAM_X_OFFSET = -3;

	private double x;
	private double y;

	private double xVel;
	private double yVel;

	private boolean valid;

	public double getX() {
		return x;
	}

	public double getY() {
		return y;
	}

	public double getXVel() {
		return xVel;
	}

	public double getYVel() {
		return yVel;
	}

	public boolean isValid() {
		return valid;
	}

	public TargetPos getPosFromLimelight() {
		return getPos(Limelight.getTX(), Limelight.getTY(), Limelight.isTargetValid());
	}

	public TargetPos getPos(double tX, double tY, boolean tV) {
		double yAngle = 90 + CAM_ANGLE + tY;

		// System.out.println(yAngle);

		if (yAngle > 89 || !tV) // Failsafe in case the angle is above the expected threshold
		{
			valid = false;
			return this;
		}
		valid = true;

		double newY = Math.tan(MathHelper.deg2Rad(yAngle)) * CAM_HEIGHT;

		double newX = Math.tan(MathHelper.deg2Rad(tX)) * y + CAM_X_OFFSET;

		xVel = newX - x;
		yVel = newY - y;

		x = newX;
		y = newY;

		return this;
	}
}