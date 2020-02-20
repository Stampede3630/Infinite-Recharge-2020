/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;

/**
 * Add your docs here.
 */
public class Limelight {
	private static NetworkTable table;

	static {
		table = NetworkTableInstance.getDefault().getTable("limelight");
	}

	public enum LedMode {
		Current, ForceOff, ForceBlink, ForceOn,
	}

	public static class Target {

		public enum TargetType {
			Undefined, PowerCell, UpperTarget
		}

		public static final double CAM_X_OFFSET = -3;

		private static double x;
		private static double y;

		private static double xVel;
		private static double yVel;

		private static double angle;

		private static boolean valid;

		private static TargetType trackedTargetType = TargetType.Undefined;

		/**
		 * Returns relative target X. If invalid, returns the last value.
		 * @return Relative target X offset (positive to the right [VERIFY]).
		 */
		public static double getX() {
			return x;
		}

		/**
		 * Returns relative target Y. If invalid, returns the last value.
		 * @return Relative target Y offset (positive away).
		 */
		public static double getY() {
			return y;
		}

		/**
		 * Returns relative target angle in degrees. If invalid, returns the last value.
		 * @return Relative target angle in degrees (positive counterclockwise [VERIFY]).
		 */
		public static double getAngle() {
			return angle;
		}

		/**
		 * Returns relative target X velocity. If invalid, returns the last value.
		 * <p>
		 * Doesn't take the robot's velocity into account.
		 * <p>
		 * This value is 0 on the first frame a new object is tracked.
		 * @return Relative target X velocity (positive to the right [VERIFY]).
		 */
		public static double getXVel() {
			return xVel;
		}

		/**
		 * Returns relative target Y velocity. If invalid, returns the last value.
		 * <p>
		 * Doesn't take the robot's velocity into account.
		 * <p>
		 * This value is 0 on the first frame a new object is tracked.
		 * @return Relative target Y velocity (positive to the right [VERIFY]).
		 */
		public static double getYVel() {
			return yVel;
		}

		/**
		 * Returns whether the target is valid.
		 * <p>
		 * The target has to be both visible and below/above the horizon appropriately (based on the input Camera Offset value).
		 * <p>
		 * Values are not updated while the target is invalid
		 * @return True if the target is valid.
		 */
		public static boolean isValid() {
			return valid;
		}

		public static TargetType getTrackedTargetType() {
			return trackedTargetType;
		}

		public static void reset() {
			x = 0;
			y = 0;
			xVel = 0;
			yVel = 0;
			angle = 0;
			valid = false;
			trackedTargetType = TargetType.Undefined;
		}

		public static boolean trackTarget(TargetType targetType) {
			if (trackedTargetType != targetType) {
				reset();
				trackedTargetType = targetType;

				// TODO: Manually set the pipeline too?
			}

			// TODO: set camAngle based on the actual camera angle (using the servo/encoder)

			switch (targetType) {
			case Undefined:
				break;
			case PowerCell:
				getPos(-30, -33);
				break;
			case UpperTarget:
				getPos(0, 60); // TODO: Verify and set actual values
				break;
			}
			return isValid();
		}

		private static void getPos(double camAngle, double verticalOffset) {
			double yAngle = 90 + camAngle + Limelight.getTY();

			// The target is invalid if it is:
			// Not visible,
			// Above the horizon when verticalOffset is < 0, or
			// below the horizon when verticalOffset it > 0
			if (!Limelight.isTargetValid() || (verticalOffset < 0 && yAngle > 89.99)
					|| (verticalOffset > 0 && yAngle < 89.99)) {
				valid = false;
				return;
			}

			double newY = Math.tan(MathHelper.deg2Rad(yAngle)) * -verticalOffset;

			double newX = Math.tan(MathHelper.deg2Rad(Limelight.getTX())) * y + CAM_X_OFFSET;


			// Velocities get updated only if the target was previously valid
			if (valid)
			{
				xVel = newX - x;
				yVel = newY - y;
			}
			else
			{
				xVel = 0;
				yVel = 0;
			}

			valid = true;

			x = newX;
			y = newY;

			angle = Math.atan2(x, y) / Math.PI * 180;
		}
	}

	private static double getEntry(String entry) {
		return table.getEntry(entry).getDouble(0);
	}

	private static void setEntry(String entry, double value) {
		table.getEntry(entry).setNumber(value);
	}

	/**
	 * tv == 1
	 * <p>
	 * Whether the limelight has any valid targets.
	 * 
	 * @return True if the limelight has a valid target.
	 */
	public static boolean isTargetValid() {
		return getEntry("tv") == 1;
	}

	/**
	 * tx
	 * <p>
	 * Horizontal offset from Crosshair to Target.
	 * 
	 * @return Angle (-29.8 to 29.8) degrees, 0 if no target is present.
	 */
	public static double getTX() {
		return getEntry("tx");
	}

	/**
	 * ty
	 * <p>
	 * Vertical offset from Crosshair to Target.
	 * 
	 * @return Angle (-24.85 to 24.85) degrees, 0 if no target is present.
	 */
	public static double getTY() {
		return getEntry("ty");
	}

	/**
	 * ta
	 * <p>
	 * Target area (% of the image).
	 * 
	 * @return Number (0 to 100) (0% to 100% of the image), 0 if no target is
	 *         present.
	 */
	public static double getTA() {
		return getEntry("ta");
	}

	/**
	 * ts
	 * <p>
	 * Target's skew or rotation.
	 * 
	 * @return Angle (-90 to 0) degrees, 0 if no target is present.
	 */
	public static double getTS() {
		return getEntry("ts");
	}

	/**
	 * tl
	 * <p>
	 * The pipeline's latency contribution (ms). Add at least 11ms for image capture
	 * latency.
	 * 
	 * @return The pipeline's latency contribution in ms.
	 */
	public static double getTL() {
		return getEntry("tl");
	}

	/**
	 * tshort
	 * <p>
	 * Sidelength of shortest side of the fitted bounding box (pixels).
	 * 
	 * @return Length of the shortest side of the target's fitted bounding box in
	 *         pixels (not the rough bounding box).
	 */
	public static double getTShort() {
		return getEntry("tshort");
	}

	/**
	 * tlong
	 * <p>
	 * Sidelength of longest side of the fitted bounding box (pixels).
	 * 
	 * @return Length of the longest side of the target's fitted bounding box in
	 *         pixels (not the rough bounding box).
	 */
	public static double getTLong() {
		return getEntry("tlong");
	}

	/**
	 * thor
	 * <p>
	 * Horizontal sidelength of the rough bounding box (pixels).
	 * 
	 * @return Horizontal length (0 - 320) of the target's rough bounding box in
	 *         pixels.
	 */
	public static double getTHor() {
		return getEntry("thor");
	}

	/**
	 * tvert
	 * <p>
	 * Vertical sidelength of the rough bounding box (pixels).
	 * 
	 * @return Vertical length (0 - 320) of the target's rough bounding box in
	 *         pixels.
	 */
	public static double getTVert() {
		return getEntry("tvert");
	}

	/**
	 * getPipe
	 * <p>
	 * True active pipeline index of the camera.
	 * 
	 * @return The active pipeline index (0 to 9).
	 */
	public static double getPipe() {
		return getEntry("getPipe");
	}

	/**
	 * tx0
	 * <p>
	 * Raw screenspace X value of target 0.
	 * 
	 * @return Normalized X value (-1 to 1) of target 0.
	 */
	public static double getTX0() {
		return getTXN(0);
	}

	/**
	 * ty0
	 * <p>
	 * Raw screenspace Y value of target 0.
	 * 
	 * @return Normalized Y value (-1 to 1) of target 0.
	 */
	public static double getTY0() {
		return getTYN(0);
	}

	/**
	 * tx1
	 * <p>
	 * Raw screenspace X value of target 1.
	 * 
	 * @return Normalized X value (-1 to 1) of target 1.
	 */
	public static double getTX1() {
		return getTXN(1);
	}

	/**
	 * ty1
	 * <p>
	 * Raw screenspace Y value of target 1.
	 * 
	 * @return Normalized Y value (-1 to 1) of target 1.
	 */
	public static double getTY1() {
		return getTYN(1);
	}

	/**
	 * tx2
	 * <p>
	 * Raw screenspace X value of target 2.
	 * 
	 * @return Normalized X value (-1 to 1) of target 2.
	 */
	public static double getTX2() {
		return getTXN(2);
	}

	/**
	 * ty2
	 * <p>
	 * Raw screenspace Y value of target 2.
	 * 
	 * @return Normalized Y value (-1 to 1) of target 2.
	 */
	public static double getTY2() {
		return getTYN(2);
	}

	/**
	 * tx< n >
	 * <p>
	 * Raw screenspace X value of target N.
	 * <p>
	 * Works for values 0, 1, and 2.
	 * 
	 * @param n Target index
	 * @return Normalized X value (-1 to 1) of target N.
	 */
	public static double getTXN(int n) {
		return getEntry("tx" + n);
	}

	/**
	 * ty< n >
	 * <p>
	 * Raw screenspace Y value of target N.
	 * <p>
	 * Works for values 0, 1, and 2.
	 * 
	 * @param n Target index
	 * @return Normalized Y value (-1 to 1) of target N.
	 */
	public static double getTYN(int n) {
		return getEntry("ty" + n);
	}

	/**
	 * getpipe
	 * <p>
	 * Raw screenspace X value of target N.
	 * <p>
	 * Works for values 0, 1, and 2.
	 * 
	 * @param n Target index
	 * @return Normalized Y value (-1 to 1) of target N.
	 */
	public static double getPipeline() {
		return getEntry("getpipe");
	}

	/**
	 * pipeline
	 * <p>
	 * Sets the current pipeline.
	 * 
	 * @param pipeline Target pipeline (0-9 inclusive).
	 */
	public static void setPipeline(int pipeline) {
		setEntry("pipeline", pipeline);
	}

	/**
	 * ledMode
	 * <p>
	 * Sets the LED mode.
	 * 
	 * @param ledMode New LED mode.
	 */
	public static void setLED(LedMode ledMode) {
		switch (ledMode) {
		case Current:
			setEntry("ledMode", 0);
			break;
		case ForceOff:
			setEntry("ledMode", 1);
			break;
		case ForceBlink:
			setEntry("ledMode", 2);
			break;
		case ForceOn:
			setEntry("ledMode", 3);
			break;
		}
	}

	/**
	 * camMode 0
	 * <p>
	 * Enables vision processing.
	 */
	public static void enableVisionProcessing() {
		setEntry("camMode", 0);
	}

	/**
	 * camMode 1
	 * <p>
	 * Disables vision processing and enables the Driver camera (longer exposure).
	 */
	public static void disableVisionProcessing() {
		setEntry("camMode", 1);
	}
}
