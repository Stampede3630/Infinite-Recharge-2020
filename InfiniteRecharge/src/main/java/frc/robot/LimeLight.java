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
public class LimeLight
{
    private static NetworkTable table;

    static
    {
        table = NetworkTableInstance.getDefault().getTable("limelight");
    }

    public enum ledMode{
        Current,
        ForceOff,
        ForceOn,
        ForceLeft,
        ForceRight
    }

    private static double getEntry(String entry)
    {
        return table.getEntry(entry).getDouble(0);
    }

    private static void setEntry(String entry, double value)
    {
        table.getEntry(entry).setNumber(value);
    }

    /**
     * Whether the limelight has any valid targets.
     * @return True if the limelight has a valid target.
     */
    public static boolean isTargetValid()
    {
        return getEntry("tv") == 1;
    }
    
    /**
     * Horizontal offset from Crosshair to Target.
     * @return Angle (-29.8 to 29.8) degrees, 0 if no target is present.
     */
    public static double getTX()
    {
        return getEntry("tx");
    }

    /**
     * Vertical offset from Crosshair to Target.
     * @return Angle (-24.85 to 24.85) degrees, 0 if no target is present.
     */
    public static double getTY()
    {
        return getEntry("ty");
    }

    /**
     * Target area (% of the image).
     * @return Number (0 to 100) (0% to 100% of the image), 0 if no target is present.
     */
    public static double getTA()
    {
        return getEntry("ta");
    }

    /**
     * Target's skew or rotation.
     * @return Angle (-90 to 0) degrees, 0 if no target is present.
     */
    public static double getTS()
    {
        return getEntry("ts");
    }

    /**
     * The pipeline's latency contribution (ms).
     * Add at least 11ms for image capture latency.
     * @return The pipeline's latency contribution in ms.
     */
    public static double getTL()
    {
        return getEntry("tl");
    }

    /**
     * Sidelength of shortest side of the fitted bounding box (pixels).
     * @return Length of the shortest side of the target's fitted bounding box in pixels (not the rough bounding box).
     */
    public static double getTShort()
    {
        return getEntry("tshort");
    }

    /**
     * Sidelength of longest side of the fitted bounding box (pixels).
     * @return Length of the longest side of the target's fitted bounding box in pixels (not the rough bounding box).
     */
    public static double getTLong()
    {
        return getEntry("tlong");
    }

    /**
     * Horizontal sidelength of the rough bounding box (pixels).
     * @return Horizontal length (0 - 320) of the target's rough bounding box in pixels.
     */
    public static double getTHor()
    {
        return getEntry("thor");
    }

    /**
     * Vertical sidelength of the rough bounding box (pixels).
     * @return Vertical length (0 - 320) of the target's rough bounding box in pixels.
     */
    public static double getTVert()
    {
        return getEntry("tvert");
    }

    /**
     * True active pipeline index of the camera.
     * @return The active pipeline index (0 to 9).
     */
    public static double getPipe()
    {
        return getEntry("getPipe");
    }

    /**
     * Raw screenspace X value of target 0.
     * @return Normalized X value (-1 to 1) of target 0.
     */
    public static double getTX0()
    {
        return getTXN(0);
    }
    
    /**
     * Raw screenspace Y value of target 0.
     * @return Normalized Y value (-1 to 1) of target 0.
     */
    public static double getTY0()
    {
        return getTYN(0);
    }

    /**
     * Raw screenspace X value of target 1.
     * @return Normalized X value (-1 to 1) of target 1.
     */
    public static double getTX1()
    {
        return getTXN(1);
    }
    
    /**
     * Raw screenspace Y value of target 1.
     * @return Normalized Y value (-1 to 1) of target 1.
     */
    public static double getTY1()
    {
        return getTYN(1);
    }

    /**
     * Raw screenspace X value of target 2.
     * @return Normalized X value (-1 to 1) of target 2.
     */
    public static double getTX2()
    {
        return getTXN(2);
    }
    
    /**
     * Raw screenspace Y value of target 2.
     * @return Normalized Y value (-1 to 1) of target 2.
     */
    public static double getTY2()
    {
        return getTYN(2);
    }

    public static double getTXN(int n)
    {
        return getEntry("tx" + n);
    }

    public static double getTYN(int n)
    {
        return getEntry("ty" + n);
    }

    public static double getPipeline()
    {
        return getEntry("getpipe");
    }

    public static void setPipeline(int pipeline)
    {
        setEntry("pipeline", pipeline);
    }

    public static void setLED(int ledMode)
    {
        setEntry("ledMode", ledMode);
    }

    public static void enableVisionProcessing()
    {
        setEntry("camMode", 0);
    }

    public static void disableVisionProcessing()
    {
        setEntry("camMode", 1);
    }
}