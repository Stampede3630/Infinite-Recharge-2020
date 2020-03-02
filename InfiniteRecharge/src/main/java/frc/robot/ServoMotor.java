/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/
package frc.robot;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.Servo;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class ServoMotor {
    private static ServoMotor instance;
    private static double servoState = RobotMap.StateChooser.LIMELIGHT_ANGLE;
	static {
		instance = new ServoMotor();
	}

	public static ServoMotor getInstance() {
		return instance;
    }

    private Servo servo;

    public ServoMotor() {

        servo = new Servo(0);
        SmartDashboard.putNumber("servo Value", 40);
    }

    public void ServoUp() {
        servo.set(1);
    }

    public void setServo(double angle)
    {
        servo.setAngle(angle);
    }

    public void setServoSmartDashboard()
    {
        setServo(SmartDashboard.getNumber("servo Value", 40));
    }

    public void servoPeriodic()
    {
        if(servoState != RobotMap.StateChooser.LIMELIGHT_ANGLE)
        {
            setServo(RobotMap.StateChooser.LIMELIGHT_ANGLE);
            servoState = RobotMap.StateChooser.LIMELIGHT_ANGLE;
        }
    }


}