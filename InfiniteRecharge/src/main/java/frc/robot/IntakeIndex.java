/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import javax.lang.model.util.ElementScanner6;

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

	private double beltForwardOne = -.15;  //TalonSRX speed = -.32;
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
			RobotMap.IntakeMap.PINWHEEL.set(.4); //was .2 - Andy
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
		if (RobotMap.CONTROLLER.getAButton()) {
			// System.out.println("tester");
			timer.reset();
			timer.start();
			RobotMap.IntakeMap.INTAKE_WHEELS.set(.4); //was .375
			RobotMap.IntakeMap.ARMS_SOLENOID.set(DoubleSolenoid.Value.kReverse);
			beltBackwardTriggered = 0;
			beltForwardTriggered = 0;
		}

		else {
			RobotMap.IntakeMap.INTAKE_WHEELS.set(0);
			// System.out.print(timer.get());
			RobotMap.IntakeMap.ARMS_SOLENOID.set(DoubleSolenoid.Value.kForward);

		}

		if (!bottom && RobotMap.CONTROLLER.getTriggerAxis(Hand.kLeft) > .6)
		{
			RobotMap.IntakeMap.PINWHEEL.set(.5); //was .375
		}
		else if (!bottom && breakBeam.getVeryBottom())
		{
			RobotMap.IntakeMap.PINWHEEL.set(.5);
		}
		else if (timer.get() > 0.5 || bottom || timer.get() == 0) { // if its been 1.5 sec or there's something in the bottom
			RobotMap.IntakeMap.PINWHEEL.set(0);
		}
		else {
			RobotMap.IntakeMap.PINWHEEL.set(.5); //was .375
		}


		if(RobotMap.CONTROLLER.getTriggerAxis(Hand.kLeft) > .6 // if shooter up to speed 
		&& Math.abs(Shooter.getRPM()) >= RobotMap.ShooterMap.RPM * 0.90) //.9 for short shot
		{
			RobotMap.IntakeMap.BELT.set(beltForwardTwo);
		}
		else if(bottomToMiddle)
		{
			RobotMap.IntakeMap.BELT.set(beltForwardOne);
		}
		else if(bottomMiddleToTop)
		{
			RobotMap.IntakeMap.BELT.set(beltForwardOne);
		}
		else if (middleTopRefine)
		{
			RobotMap.IntakeMap.BELT.set(beltForwardOne);
		}
		else
		{
			RobotMap.IntakeMap.BELT.set(0);
		}






/*
		If ball on bottom then move to 3 & 4
		If ball on (some form of bottom) & middle then move to 0 & 1
		If ball on bottom & some form of top (do nothing)
		If shooting trigger and up to speed … move … period
		If shooting trigger and not (some form of bottom) start pinwheel
*/


/* BELT 2
		if(RobotMap.CONTROLLER.getTriggerAxis(Hand.kLeft) > .6 // if shooter up to speed 
		&& Math.abs(Shooter.getRPM()) >= RobotMap.ShooterMap.RPM * 0.90) //.9 for short shot
		{
			RobotMap.IntakeMap.BELT.set(beltForwardTwo);
		}
		else if (bottom && weakTop)
		{
			RobotMap.IntakeMap.BELT.set(0);
		}
		else if(top)
		{
			RobotMap.IntakeMap.BELT.set(0);
		}
		else if (weakBottom && middle && !top)
		{
			RobotMap.IntakeMap.BELT.set(beltForwardOne);
		}
		/*
		else if(top && weakMiddle && !breakBeam.getBottomGap())
		{
			RobotMap.IntakeMap.BELT.set(beltForwardOne);
		}
		*/
		/*
		else if(weakBottom && !middle)
		{
			RobotMap.IntakeMap.BELT.set(beltForwardOne);
		}
	
		else if (!none && !bottom && !middle && !top)
		{
			RobotMap.IntakeMap.BELT.set(beltForwardOne);
		}
		else
		{
			RobotMap.IntakeMap.BELT.set(0);
		}

		*/
		// BELT STUFF!!!!!!!!!!!!!!!!!
		/*
		if (RobotMap.CONTROLLER.getBumper(Hand.kRight)) {
			RobotMap.IntakeMap.BELT.set(beltForwardOne);
			beltBackwardTriggered = timeout +1;
			beltForwardTriggered = timeout +1;
		} else if (RobotMap.CONTROLLER.getBumper(Hand.kLeft)) {
			RobotMap.IntakeMap.BELT.set(0);
		} else if (RobotMap.CONTROLLER.getTriggerAxis(Hand.kLeft) > .6 // if shooter up to speed 
				&& Math.abs(Shooter.getRPM()) >= RobotMap.ShooterMap.RPM * 0.85) {
			RobotMap.IntakeMap.BELT.set(beltForwardTwo);
			SmartDashboard.putNumber("shooter threshold", Shooter.getRPM());
			SmartDashboard.putNumber("Shooter value", Shooter.getRPM());

			SmartDashboard.putString("shooter state","shooter up to speed");
			beltForwardTriggered = 0;
			beltBackwardTriggered = 0;

		} 
		else if (RobotMap.CONTROLLER.getTriggerAxis(Hand.kLeft) > .6) {
			RobotMap.IntakeMap.BELT.set(0);
			SmartDashboard.putString("shooter state","shooter NOT up to speed");

		}
		else if (beltBackwardTriggered > timeout && beltForwardTriggered > timeout)
		{
			RobotMap.IntakeMap.BELT.set(0);
		}
		else if (top && middle) {
			RobotMap.IntakeMap.BELT.set(0);
			SmartDashboard.putString("intake state","yes top, yes middle");
		} else if (top && !middle) {
			RobotMap.IntakeMap.BELT.set(beltBackwardsOne);
			beltBackwardTriggered++;
			SmartDashboard.putString("intake state","yes top, no middle");

		} else if (!top && middle && bottom) {
			RobotMap.IntakeMap.BELT.set(beltForwardTwo);
			beltForwardTriggered++;
			SmartDashboard.putString("intake state","middle and bottom");

		} else if (!top && !middle && bottom) {
			RobotMap.IntakeMap.BELT.set(beltForwardOne);
			beltForwardTriggered++;
			SmartDashboard.putString("intake state","only bottom");

		} else if (middle) {
			RobotMap.IntakeMap.BELT.set(0);
			SmartDashboard.putString("intake state","middle");
		} /*else if (limbo){
			RobotMap.IntakeMap.BELT.set(beltForwardOne);
		}*/
		/*
		else if (none) {
			SmartDashboard.putString("intake state","none");
			RobotMap.IntakeMap.BELT.set(0);
		}
		/*
		 * else if (!none && !bottom && !middle && !top){
		 * RobotMap.IntakeMap.BELT.set(beltForwardOne); }
		 */

	}

	/**
	 * This belt logic looks at the current belt speed to determine the current
	 * state. If we are shooting and up to speed, we go forward. Otherwise, we use
	 * the following logic. If no balls are detected, we set the speed to 0. If the
	 * belt is moving forward, we stop when a ball reaches top. If the belt is
	 * moving backward, we stop when a ball reaches middle. If the belt is at rest,
	 * we look at the sensors. - top, middleWeak and bottom: no movement - just
	 * bottom: move forward until top is triggered (to top) - top and bottom: move
	 * backward until middle is triggered (to middle and bottom) - middle and
	 * bottom: move forward until top is triggered (to top and middle) This code
	 * assumes that the belt will return 0.0 when it is not moving. Also, should we
	 * add a timer so that the belt stops in case of errors? Maybe run for 1-2
	 * seconds at most?
	 */

	/*
	 * public void tempBelt() { // BELT STUFF!!!!!!!!!!!!!!!!! if
	 * (RobotMap.CONTROLLER.getBButton()) {
	 * RobotMap.IntakeMap.ARMS_SOLENOID.set(DoubleSolenoid.Value.kReverse); }
	 * 
	 * if (RobotMap.CONTROLLER.getTriggerAxis(Hand.kRight) > .6 // if shooter up to
	 * speed && RobotMap.ShooterMap.LEFT_SHOOTER_FALCON.getSelectedSensorVelocity()
	 * >= Shooter.rpmToRotatPer100Mili(Shooter.rotpm) Shooter.kEncoderUnitsPerRev) {
	 * RobotMap.IntakeMap.BELT.set(beltForwardTwo);
	 * SmartDashboard.putString("shooter up to speed"); } else if (none) {
	 * System.out.println("none, reset to 0"); RobotMap.IntakeMap.BELT.set(0.0); }
	 * else if (top && middleWeak && bottom) { RobotMap.IntakeMap.BELT.set(0.0);
	 * System.out.println("top, middleWeak and bottom, reset to/stay at 0"); } else
	 * if (RobotMap.IntakeMap.BELT.get() == 0.0) { // ready for new command if (top
	 * && bottom) { RobotMap.IntakeMap.BELT.set(beltBackwardsOne);
	 * System.out.println("top, bottom, go BACK"); } else if (middleWeak && bottom)
	 * { RobotMap.IntakeMap.BELT.set(beltForwardTwo);
	 * System.out.println("middle weak, bottom, go FORWARD"); } else if (bottom) {
	 * RobotMap.IntakeMap.BELT.set(beltForwardOne);
	 * System.out.println("bottom, go FORWARD"); } } else if
	 * (RobotMap.IntakeMap.BELT.get() < 0.0) { // currently moving forward if (top)
	 * { RobotMap.IntakeMap.BELT.set(0.0); System.out.println("top, STOP forward");
	 * if (middleWeak) { System.out.println("\t also middle weak"); } } } else if
	 * (RobotMap.IntakeMap.BELT.get() > 0.0) { // currently moving backward if
	 * (middle) { RobotMap.IntakeMap.BELT.set(0.0);
	 * System.out.println("middle, STOP backward"); if (bottom) {
	 * System.out.println("\t also bottom"); } } } }
	 */

	public void toSmartDashboard() {
		// SmartDashboard.putNumber("colorSensorHigh Green",
		// RobotMap.IntakeMap.COLOR_SENSOR_HIGH.getGreen());
		// SmartDashboard.putNumber("colorSensorMid Green",
		// RobotMap.IntakeMap.COLOR_SENSOR_MID.getGreen());
		//SmartDashboard.putNumber("ultrasonic", RobotMap.IntakeMap.ULTRASONIC.getRangeInches());
		SmartDashboard.putBoolean("bottom", bottom);
		SmartDashboard.putBoolean("middle", middle);
		SmartDashboard.putBoolean("top", top);
		SmartDashboard.putBoolean("none", none);
		SmartDashboard.putNumber("falcon velocity",
				RobotMap.ShooterMap.LEFT_SHOOTER_FALCON.getSelectedSensorVelocity());
		SmartDashboard.putNumber("desired velocity",
				Shooter.rpmToRotatPer100Mili(RobotMap.ShooterMap.RPM) * RobotMap.ShooterMap.ENCODER_UNITS_PER_REV);
		SmartDashboard.putBoolean("Shooter boolean", RobotMap.CONTROLLER.getTriggerAxis(Hand.kLeft) > .6 // if shooter
																											// up to
																											// speed
				&& Math.abs(RobotMap.ShooterMap.LEFT_SHOOTER_FALCON.getSelectedSensorVelocity(0)) >= Shooter
						.rpmToRotatPer100Mili(RobotMap.ShooterMap.RPM) * RobotMap.ShooterMap.ENCODER_UNITS_PER_REV);
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

		if (RobotMap.CONTROLLER.getYButtonPressed()) {

			if (hoodNewMattyState == false) {
				RobotMap.IntakeMap.HOOD_ANGLE.set(DoubleSolenoid.Value.kForward);
				hoodNewMattyState = true;

			} else if (hoodNewMattyState == true) {
				RobotMap.IntakeMap.HOOD_ANGLE.set(DoubleSolenoid.Value.kReverse);
				hoodNewMattyState = false;
			}

		}
	}

}