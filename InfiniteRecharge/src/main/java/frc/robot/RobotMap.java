/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Talon;

/**
 * Add your docs here.
 */
public class RobotMap {

    public static final Talon TALON_FR = new Talon(1);
    public static final Talon TALON_FL = new Talon(3);
    public static final Talon TALON_BR = new Talon(0);
    public static final Talon TALON_BL = new Talon(2);
    public static final Talon TALON_GEAR = new Talon(4);

    public static final DigitalInput LIMIT_GEAR_OPEN = new DigitalInput(8);
    public static final DigitalInput LIMIT_GEAR_CLOSE = new DigitalInput(9);

    public static final double GEAR_SPEED = -1;

    public static final Joystick JOYSTICK = new Joystick(5);
    
    public static final XboxController controller = new XboxController(0); 
}
