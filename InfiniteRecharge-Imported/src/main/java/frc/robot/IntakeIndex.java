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
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.DoubleSolenoid;

/**
 * Add your docs here.
 */
// Use twoBeltIndex for anything with buttons and belts
public class IntakeIndex {

	private static IntakeIndex instance;
	private DigitalInput topButton = new DigitalInput(9); // port 1
	private DigitalInput middleButton = new DigitalInput(7); // 
	private DigitalInput bottomButton = new DigitalInput(8); //port 2 ?
	private DigitalInput spikeyButton = new DigitalInput(6); //DNE

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
	private double beltForwardTwo = -.6; //TalonSRX speed = -.5;
	private double beltBackwardsTwo = .5; //TalonSRX speed = .5;
	private int beltForwardTriggered = 0;
	private int beltBackwardTriggered = 0;
	private double pinwheelForward = -.6;
	private int timeout = 999999999;
	private double intakeForward = .4;

	private boolean ballStopper = false;
	private boolean twoBalls = false;

	private IntakeIndex() {
		timer = new Timer();
		breakBeam = BreakBeam.getInstance();
		indexYes = false;
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
			RobotMap.IntakeMap.INTAKE_WHEELS.set(.5);//.75
			RobotMap.IntakeMap.PINWHEEL.set(.2); //was .2 - Andy
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

	public void twoBeltTwoBallIndex(){ //THE RIGHT INTAKE the one where it "bops up" when shooter is pressed 

		setBallStopper();
		
		if(RobotMap.CONTROLLER.getTriggerAxis(Hand.kRight) >.6 || RobotMap.CONTROLLER.getTriggerAxis(Hand.kLeft) > .6) {
			indexYes = true;
			timer.reset();
			timer.start();
		} else if( timer.get() > 5) {
			indexYes = false;
		}

		
		
		if((RobotMap.CONTROLLER.getTriggerAxis(Hand.kLeft) > .6 ||  RobotMap.AutoBooleans.SHOOT_NOW)  // if left trigger is pressed and shooter up to speed 
		&& Math.abs(Shooter.getRPM()) >= RobotMap.StateChooser.RPM*.97 //* 0.90
		){
			RobotMap.IntakeMap.BELT.set(beltForwardTwo);
			RobotMap.IntakeMap.PINWHEEL.set(pinwheelForward);
		}
		else if(RobotMap.CONTROLLER.getTriggerAxis(Hand.kLeft) > .6 && RobotMap.CONTROLLER.getYButton())
		{
			RobotMap.IntakeMap.BELT.set(beltForwardTwo);
		}
		else if (RobotMap.CONTROLLER.getTriggerAxis(Hand.kLeft) > .6 && !topButton.get() && !bottomButton.get()){
			RobotMap.IntakeMap.BELT.set(0);
			RobotMap.IntakeMap.PINWHEEL.set(pinwheelForward);
		}
		else if (RobotMap.CONTROLLER.getBumper(Hand.kLeft)) //if left bumper, forwards belt
		{
			RobotMap.IntakeMap.PINWHEEL.set(-pinwheelForward);
		}
		else if (RobotMap.CONTROLLER.getBumper(Hand.kRight)){ //if right bumper, forward pinwheel
			RobotMap.IntakeMap.PINWHEEL.set(pinwheelForward);
		}
		else if(!topButton.get() && !bottomButton.get() && middleButton.get()){ //if top and bottom, move bottom to middle 
			RobotMap.IntakeMap.BELT.set(0);
			RobotMap.IntakeMap.PINWHEEL.set(pinwheelForward);
		}
		/*
		else if(!topButton.get() && !bottomButton.get()){ //if top and bottom, stop completely 
			RobotMap.IntakeMap.BELT.set(0);
			RobotMap.IntakeMap.PINWHEEL.set(0);
		}*/
		else if (!topButton.get() && !middleButton.get()){ 
			RobotMap.IntakeMap.BELT.set(0);
			RobotMap.IntakeMap.PINWHEEL.set(0); //maybe have it stop or be slower ??????
		}
		else if(!topButton.get()) 
		{
			
				RobotMap.IntakeMap.BELT.set(0);
			RobotMap.IntakeMap.PINWHEEL.set(pinwheelForward);
			
		}
		else{
			
						RobotMap.IntakeMap.BELT.set(beltForwardTwo);
				RobotMap.IntakeMap.PINWHEEL.set(pinwheelForward);
			}
		// else {
		// 	RobotMap.IntakeMap.BELT.set(0);
		// 	RobotMap.IntakeMap.PINWHEEL.set(0);
		// }

		//INTAKE 
		if(RobotMap.IntakeMap.INTAKE_WHEELS.getOutputCurrent() > 47){
			RobotMap.IntakeMap.INTAKE_WHEELS.set(0);
		}
		else if (RobotMap.CONTROLLER.getTriggerAxis(Hand.kRight) > 0.6 || RobotMap.AutoBooleans.INTAKE_NOW) {
			RobotMap.IntakeMap.INTAKE_WHEELS.set(.5); //was .375
			RobotMap.IntakeMap.ARMS_SOLENOID.set(DoubleSolenoid.Value.kReverse);
			beltBackwardTriggered = 0;
			beltForwardTriggered = 0;
		}
		else if(RobotMap.CONTROLLER.getStickButton(Hand.kRight))
		{
			RobotMap.IntakeMap.INTAKE_WHEELS.set(.8);
		}
		else if(RobotMap.CONTROLLER.getStickButton(Hand.kLeft))
		{
			RobotMap.IntakeMap.INTAKE_WHEELS.set(-.8);
		}
		else {
			RobotMap.IntakeMap.INTAKE_WHEELS.set(0);
			RobotMap.IntakeMap.ARMS_SOLENOID.set(DoubleSolenoid.Value.kForward);
		}
		
	}

	public void autoIntake(){ //the one where it "bops up" when shooter is pressed, but for auto

		if(RobotMap.AutoBooleans.SHOOT_NOW  // if left trigger is pressed and shooter up to speed 
		&& Math.abs(Shooter.getRPM()) >= RobotMap.StateChooser.RPM*.87 //* 0.90
		)
		{
			RobotMap.IntakeMap.BELT.set(beltForwardTwo);
			RobotMap.IntakeMap.PINWHEEL.set(pinwheelForward);
		}
		else if(!topButton.get() && !bottomButton.get()){ //if top and bottom, stop completely 
			RobotMap.IntakeMap.BELT.set(0);
			RobotMap.IntakeMap.PINWHEEL.set(0);
		}
		else if (!topButton.get() && !middleButton.get()){ 
			RobotMap.IntakeMap.BELT.set(0);
			RobotMap.IntakeMap.PINWHEEL.set(0); //maybe have it stop or be slower ??????
		}
		else if(!topButton.get()) 
		{
			RobotMap.IntakeMap.BELT.set(0);
			RobotMap.IntakeMap.PINWHEEL.set(pinwheelForward);
		}
		else{
			RobotMap.IntakeMap.BELT.set(beltForwardTwo);
			RobotMap.IntakeMap.PINWHEEL.set(pinwheelForward);
		}
		/*
		if(RobotMap.AutoBooleans.INTAKE_NOW = true){
			RobotMap.IntakeMap.INTAKE_WHEELS.set(.8); //was .375
			RobotMap.IntakeMap.ARMS_SOLENOID.set(DoubleSolenoid.Value.kReverse);
		}
		else if(RobotMap.AutoBooleans.INTAKE_NOW = false){
			RobotMap.IntakeMap.INTAKE_WHEELS.set(0);
			RobotMap.IntakeMap.ARMS_SOLENOID.set(DoubleSolenoid.Value.kForward);
		}*/

	
}
	
	//DEBUGGING 1/20/2021
	public void showButtons(){
		SmartDashboard.putBoolean("Top Button", topButton.get());
		SmartDashboard.putBoolean("Middle Button", middleButton.get());
		SmartDashboard.putBoolean("Bottom Button", bottomButton.get());
		SmartDashboard.putBoolean("Spikey Button", spikeyButton.get());
		SmartDashboard.putBoolean("twoballs", twoBalls);
	}

	public void setBallStopper(){
		if(SmartDashboard.getBoolean("Intake Ball Stopper", true)){
			ballStopper = true;
		}
		else{
			ballStopper = false;
		}
	}

	public void putBallStopper(){
		SmartDashboard.putBoolean("Intake Ball Stopper", false);

	}
}