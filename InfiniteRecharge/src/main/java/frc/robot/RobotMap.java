/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;


import edu.wpi.first.wpilibj.XboxController;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonFX;

/**
 * Add your docs here.
 */
public class RobotMap {

    public static XboxController controller = new XboxController(0);

    public static WPI_TalonFX shooter1 = new WPI_TalonFX(12); // GOOD + is right
    public static WPI_TalonFX shooter2 = new WPI_TalonFX(13);

}
