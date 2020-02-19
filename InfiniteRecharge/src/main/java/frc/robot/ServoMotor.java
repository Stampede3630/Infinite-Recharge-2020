/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Servo;

/**
 * Add your docs here.
 */
public class ServoMotor {

    Servo servo1;
    double ty = NetworkTableInstance.getDefault().getTable("limelight").getEntry("ty").getDouble(0);
    double tv = NetworkTableInstance.getDefault().getTable("limelight").getEntry("tv").getDouble(0);

    public ServoMotor() {

        servo1 = new Servo(0);

        if (tv == 0) {
            ty -= 0.5;

            if (ty < -1) {
                ty += 0.5;
            }

            if (ty > 1) {
                ty -= 0.5;
            }
        }
    }

    public void ServoUp() {
        servo1.set(1);
    }

    public void ServoForward() {
        servo1.set(0.5);
    }

    public void ServoBackwards() {
        servo1.set(0);
    }
}