/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot;

import edu.wpi.first.wpilibj.Servo;

/**
 * Add your docs here.
 */
public class ServoTest {

    Servo servo1;
public ServoTest() {

        servo1 = new Servo(0);
    }

public void TestServo () {

        servo1.set(0.5);
    }

}
