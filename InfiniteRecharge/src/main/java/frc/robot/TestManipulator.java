/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.NeutralMode;
import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;

import edu.wpi.first.wpilibj.Counter;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

/**
 * Add your docs here.
 */
public class TestManipulator {
	Counter counter = new Counter(4);
	WPI_TalonSRX intake = new WPI_TalonSRX(9);
	WPI_TalonSRX belt = new WPI_TalonSRX(11);
	WPI_TalonSRX highRoller = new WPI_TalonSRX(10);

	public TestManipulator() {
		belt.setNeutralMode(NeutralMode.Brake);
		counter.setSemiPeriodMode(true);
	}

	public void periodic() {
		if (Robot.m_controller.getAButton()) {
			intake.set(.75);
			highRoller.set(-.4);
		} else {
			intake.set(0);
			highRoller.set(0);
		}
		if (Robot.m_controller.getTriggerAxis(Hand.kRight) > 0.5) {
			belt.set(-.7);
		} else if (Robot.m_controller.getBumper(Hand.kRight)) {
			belt.set(0.7);
		} else {
			belt.set(0);
		}
		/*
		 * if(Robot.m_controller.getXButton()) { highRoller.set(-.8); } else {
		 * highRoller.set(0); }
		 */
	}

	public void getUltra() {

		SmartDashboard.putNumber("Ultrasonic Period", counter.getPeriod());

	}
}
