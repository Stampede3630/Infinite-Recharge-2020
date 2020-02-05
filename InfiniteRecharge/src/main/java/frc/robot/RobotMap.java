/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.Talon;

/**
 * Add your docs here.
 */
public class RobotMap {

<<<<<<< HEAD
    /* climbing stuff, needs real ports
    public static CANSparkMax trolleySpark = new CANSparkMax(99, MotorType.kBrushless);
    public static CANSparkMax elevatorSpark = new CANSparkMax(98, MotorType.kBrushless);

    public static DigitalInput elevatorMaxExtension = new DigitalInput(20);
    public static DigitalInput elevatorMinExtension = new DigitalInput(19);
    */

    public static XboxController controller = new XboxController(0);

    public static final AHRS m_gyro = new AHRS(SPI.Port.kMXP);
    
    // Swerve hardware
    
    //PID Constants/Contraints
    public static final double kMaxSpeed = 4; // 3 meters per second
    public static final double kMaxAngularSpeed = Math.PI;
    
   
    private static RobotMap thisInstance;


    public RobotMap() {
        
=======
    public static final Talon TALON_FR = new Talon(1);
    public static final Talon TALON_FL = new Talon(3);
    public static final Talon TALON_BR = new Talon(0);
    public static final Talon TALON_BL = new Talon(2);
    
    public static final Talon TALON_GEAR = new Talon(4);
>>>>>>> d6431b9f10f6bbf78dec9e5c8c1df2ff4c323960

    public static final DigitalInput LIMIT_GEAR_OPEN = new DigitalInput(8);
    public static final DigitalInput LIMIT_GEAR_CLOSE = new DigitalInput(9);

    public static final double GEAR_SPEED = -1;

<<<<<<< HEAD
        }
        return thisInstance;
    }
    
=======
    public static final Joystick JOYSTICK = new Joystick(5);
>>>>>>> d6431b9f10f6bbf78dec9e5c8c1df2ff4c323960
}
