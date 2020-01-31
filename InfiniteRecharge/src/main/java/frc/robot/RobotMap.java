/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.XboxController;

/**
 * Add your docs here.
 */
public class RobotMap {

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
        

    }

    public static RobotMap getInstance() {
        if (thisInstance == null) {
            thisInstance = new RobotMap();

        }
        return thisInstance;
    }
    
}
