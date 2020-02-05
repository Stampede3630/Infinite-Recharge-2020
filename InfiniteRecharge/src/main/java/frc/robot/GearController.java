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
public class GearController {

    public static void open() {
        if (RobotMap.LIMIT_GEAR_OPEN.get()) {
            RobotMap.TALON_GEAR.set(-RobotMap.GEAR_SPEED);
        } else
            stop();
    }
    
    public static void close() {
        if (RobotMap.LIMIT_GEAR_CLOSE.get()) {
            RobotMap.TALON_GEAR.set(RobotMap.GEAR_SPEED);
        } else
            stop();
    }

    public static void stop() {
        RobotMap.TALON_GEAR.set(0);
    }
}
