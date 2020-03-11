/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


import edu.wpi.first.wpilibj.DoubleSolenoid;

/**
 * Add your docs here.
 */

public class IntakeIndex {

	private static IntakeIndex instance;

	static {
		instance = new IntakeIndex();
	}

	public static IntakeIndex getInstance() {
		return instance;
	}

	private SendableChooser<Integer> intakeChooser;
	private Timer timer;
	private Shooter shoot;
	private boolean bottom, middle, top, weakTop, weakBottom, none, weakMiddle, limbo, bottomToMiddle, bottomMiddleToTop, middleTopRefine;
	private boolean indexYes;

	private BreakBeam breakBeam;
	private boolean newMattyState = false;
	private boolean hoodNewMattyState = false;

	private double beltForwardOne = -.2;  //TalonSRX speed = -.32;
	private double beltBackwardsOne = .15;// TalonSRX speed = .32;
	private double beltForwardTwo = -.4; //TalonSRX speed = -.5;
	private double beltBackwardsTwo = .4; //TalonSRX speed = .5;
	private int beltForwardTriggered = 0;
	private int beltBackwardTriggered = 0;
	private int timeout = 999999999;

	private IntakeIndex() {
		timer = new Timer();
		breakBeam = BreakBeam.getInstance();
		indexYes = true;
	}

	public void updateBooleans() {
		bottom = breakBeam.detectBallLow();
		middle = breakBeam.detectBallMid();
		top = breakBeam.detectBallHigh();
		none = breakBeam.noBalls();
		weakBottom = breakBeam.detectWeakBottom();
		weakTop = breakBeam.detectWeakTop();
		weakMiddle = breakBeam.detectWeakMiddle();
		bottomToMiddle = breakBeam.bottomToMiddle();
		bottomMiddleToTop = breakBeam.bottomMiddleToTop();
		middleTopRefine = breakBeam.middleTopRefine();
	}

	public void intakeChooser(boolean indexYes) {
		if (RobotMap.CONTROLLER.getStartButtonPressed() && !indexYes) {
			index();
			indexYes = true;
		}
		if (RobotMap.CONTROLLER.getStartButtonPressed() && indexYes) {
			manualControl();
			indexYes = false;
		}
	}

	public void manualControl() {
		if (RobotMap.CONTROLLER.getAButton()) {
			RobotMap.IntakeMap.INTAKE_WHEELS.set(.8);//.75
			RobotMap.IntakeMap.PINWHEEL.set(.7); //was .2 - Andy
		} else {
			RobotMap.IntakeMap.INTAKE_WHEELS.set(0);
			RobotMap.IntakeMap.PINWHEEL.set(0);
		}

		if (RobotMap.CONTROLLER.getTriggerAxis(Hand.kRight) > 0.5) {
			RobotMap.IntakeMap.BELT.set(-.7);
		} else if (RobotMap.CONTROLLER.getBumper(Hand.kRight)) {
			RobotMap.IntakeMap.BELT.set(0.7);
		} else {
			RobotMap.IntakeMap.BELT.set(0);
		}

	}

	public void index() { // most recent intake machine
		// System.out.println(pinwheel.get());
		updateBooleans();
		// System.out.println(RobotMap.ShooterMap.LEFT_SHOOTER_FALCON.getSelectedSensorVelocity());
		if (RobotMap.CONTROLLER.getTriggerAxis(Hand.kRight) > 0.6 || RobotMap.AutoBooleans.INTAKE_NOW) {
			// System.out.println("tester");
			timer.reset();
			timer.start();
			RobotMap.IntakeMap.INTAKE_WHEELS.set(.6); //was .375
			RobotMap.IntakeMap.ARMS_SOLENOID.set(DoubleSolenoid.Value.kReverse);
			beltBackwardTriggered = 0;
			beltForwardTriggered = 0;
		}
		else {
			RobotMap.IntakeMap.INTAKE_WHEELS.set(0);
			// System.out.print(timer.get());
			RobotMap.IntakeMap.ARMS_SOLENOID.set(DoubleSolenoid.Value.kForward);

		}
		
		if(RobotMap.CONTROLLER.getBumper(Hand.kRight))
		{
			RobotMap.IntakeMap.PINWHEEL.set(-.6);
		}
		else if (BreakBeam.getInstance().detectSpikyBottomBall())
		{
			RobotMap.IntakeMap.PINWHEEL.set(0);
		}
		else if (BreakBeam.getInstance().detectSpikyBottomBall()){
			RobotMap.IntakeMap.PINWHEEL.set(0);
		}
		else if (!BreakBeam.getInstance().detectSpikyBottomBall() && (RobotMap.CONTROLLER.getTriggerAxis(Hand.kLeft) > .6 || RobotMap.AutoBooleans.SHOOT_NOW))
		{
			RobotMap.IntakeMap.PINWHEEL.set(.6); //was .375 // was 0.5
		}
		else if (BreakBeam.getInstance().getRampBeam())
		{
			RobotMap.IntakeMap.PINWHEEL.set(.55);
			timer.reset();
			timer.start();
		}
		else if (timer.get() > .5) {
			RobotMap.IntakeMap.PINWHEEL.set(0);
		}
		/*
		else if (!bottom && breakBeam.getVeryBottom())
		{
			RobotMap.IntakeMap.PINWHEEL.set(.45);
		}
		*/
		/*
		else if (timer.get() > 1 || BreakBeam.getInstance().detectSpikyBottomBall() || timer.get() == 0) { // if its been 1.5 sec or there's something in the bottom
			RobotMap.IntakeMap.PINWHEEL.set(0);
		}
		else {
			RobotMap.IntakeMap.PINWHEEL.set(.55); //was .375
		}
		*/
		
		
		
		if((RobotMap.CONTROLLER.getTriggerAxis(Hand.kLeft) > .6 ||  RobotMap.AutoBooleans.SHOOT_NOW)  // if shooter up to speed 
		&& Math.abs(Shooter.getRPM()) >= RobotMap.StateChooser.RPM*.97 //* 0.90
		
		) //.9 for short shot
		{
			RobotMap.IntakeMap.BELT.set(beltForwardTwo);
		}
		else if (RobotMap.CONTROLLER.getBumper(Hand.kLeft))
		{
			RobotMap.IntakeMap.BELT.set(-beltForwardTwo);
		}
		else if (breakBeam.getVeryTop())
		{
			RobotMap.IntakeMap.BELT.set(0);
		}
		else if (middleTopRefine)
		{
			RobotMap.IntakeMap.BELT.set(beltForwardOne);
		}
		else if(bottomToMiddle)
		{
			RobotMap.IntakeMap.BELT.set(beltForwardOne);
		}
		else if(bottomMiddleToTop)
		{
			RobotMap.IntakeMap.BELT.set(beltForwardOne);
		}
		else
		{
			RobotMap.IntakeMap.BELT.set(0);
		}

		if (RobotMap.CONTROLLER.getXButtonPressed()) {

			if (RobotMap.StateChooser.HOOD_ANGLE == false) {
				RobotMap.StateChooser.HOOD_ANGLE = true;

			} else if (RobotMap.StateChooser.HOOD_ANGLE == true) {
				RobotMap.StateChooser.HOOD_ANGLE = false;
			}

		}
		if(RobotMap.StateChooser.HOOD_ANGLE) //MAKE SURE NOT BAD
		{
			RobotMap.IntakeMap.HOOD_ANGLE.set(Value.kForward);
		}
		else
		{
			RobotMap.IntakeMap.HOOD_ANGLE.set(Value.kReverse);
		}
	}

	public void ToggleSolenoids() {
		if (RobotMap.CONTROLLER.getXButtonPressed()) {

			if (newMattyState == false) {
				RobotMap.IntakeMap.ARMS_SOLENOID.set(DoubleSolenoid.Value.kForward);
				newMattyState = true;

			} else if (newMattyState == true) {
				RobotMap.IntakeMap.ARMS_SOLENOID.set(DoubleSolenoid.Value.kReverse);
				newMattyState = false;
			}

		}
	}

}