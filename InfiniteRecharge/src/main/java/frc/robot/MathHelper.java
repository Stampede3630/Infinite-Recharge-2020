/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

/**
 * Add your docs here.
 */
public class MathHelper {
    
    public static double clamp(double min, double max, double value) {
        return Math.min(max, Math.max(min, value));
    }

    public static double clampUnit(double value) {
        return clamp(-1, 1, value);
    }

    public static double clamp01(double value) {
        return clamp(0, 1, value);
    }

    public static double deg2Rad(double degrees)
    {
        return degrees / 180 * Math.PI;
    }

    public static double rad2Deg(double radians)
    {
        return radians / Math.PI * 180;
    }

    public static double deadzone(double value, double deadzone)
    {
        if (value <= deadzone && value >= -deadzone) return 0;

        return clamp01((Math.abs(value) - deadzone) / (1 - deadzone)) * Math.signum(value);
    }

    public static double lerpUnclamped(double value, double min, double max)
    {
        return min + value * (max - min);
    }

    public static double lerp(double value, double min, double max)
    {
        return lerpUnclamped(clamp01(value), min, max);
    }

    public static double remapUnclamped(double value, double minValue, double maxValue, double minOutput, double maxOutput)
    {
        return lerp((value - minValue) / maxValue, minOutput, maxOutput);
    }

    public static double remap(double value, double minValue, double maxValue, double minOutput, double maxOutput)
    {
        return remapUnclamped(clamp(value, minValue, maxValue), minValue, maxValue, minOutput, maxOutput);
    }
}