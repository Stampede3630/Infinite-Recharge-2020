/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class AVA {
Encoder cod;
double trouble;
PIDController piddie; 


public AVA()
{
cod = new Encoder(0,1); 

}
public void piggi()

{

    SmartDashboard.putNumber("encoder value", cod.getDistance());
 trouble = cod.getDistance()/250*Math.PI*2*2.5;
 
 
}
public double piggie()

 { return cod.getDistance()/250*Math.PI*2*2.5;

}

}
