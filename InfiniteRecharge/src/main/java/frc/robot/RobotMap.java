/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;

import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.XboxController;

/**
 * Add your docs here.
 */
public class RobotMap {
    //public static CANSparkMax trolleySpark = new CANSparkMax(8, MotorType.kBrushless);
    public static CANSparkMax elevatorSpark = new CANSparkMax(14, MotorType.kBrushless);

    public static DigitalInput maxLimitSwitch = new DigitalInput(20);
    public static DigitalInput minLimitSwitch = new DigitalInput(19);

    public static XboxController controller = new XboxController(0);
}
