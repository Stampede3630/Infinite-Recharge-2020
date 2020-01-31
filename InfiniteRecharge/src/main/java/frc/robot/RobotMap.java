/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018 FIRST. All Rights Reserved.                             */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import edu.wpi.first.wpilibj.Ultrasonic;
import edu.wpi.first.wpilibj.*;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.buttons.JoystickButton;
import edu.wpi.first.wpilibj.drive.MecanumDrive;
import com.ctre.phoenix.CANifier;
public class RobotMap {
	
	public final Compressor mainC = new Compressor(0);
	
	public final DoubleSolenoid solenoidBack = new DoubleSolenoid(6,7);
	public final DoubleSolenoid solenoidFront = new DoubleSolenoid(4,5);
	public final DoubleSolenoid hatchExtend = new DoubleSolenoid(0,0,1); //0,2

    public final WPI_TalonSRX talonFL =  new WPI_TalonSRX (4); //4
    public final WPI_TalonSRX talonFR =  new WPI_TalonSRX (9); //3
    public final WPI_TalonSRX talonBL =  new WPI_TalonSRX (2); //2
	public final WPI_TalonSRX talonBR =  new WPI_TalonSRX (1); //1
	public final WPI_TalonSRX talonHatchR = new WPI_TalonSRX(3);
	public final WPI_TalonSRX talonHatchL = new WPI_TalonSRX(5);
	public final WPI_TalonSRX talonBallShooter = new WPI_TalonSRX(6); //6
	//public final WPI_TalonSRX talonBallIntake = new WPI_TalonSRX (7); //5, practice bot ///COMMENT OUT FOR COMP
	
	public final CANSparkMax talonBallIntake = new CANSparkMax(8, MotorType.kBrushless);

    public final MecanumDrive drive = new MecanumDrive(talonFL, talonBL, talonFR, talonBR);

    public final AHRS ahrs = new AHRS (SPI.Port.kMXP);
  
	public final DigitalInput ballButton = new DigitalInput(0);
	public final DigitalInput lowBallButton = new DigitalInput(1);
	public final DigitalInput hatchButton = new DigitalInput(2);
	public final DigitalInput dumbHatchButton = new DigitalInput(3);//change?

	public final AnalogInput pressureLevel = new AnalogInput(0); //3

 
//	public final DoubleSolenoid hatchDeploy = new DoubleSolenoid(2,3); // 1,3

	public final Ultrasonic ultrasonicSensor = new Ultrasonic(9,8);
	
	//public final WPI_TalonSRX slideTalon = new WPI_TalonSRX(5);

	public final CANifier canifier = new CANifier(0);


    //XboxController

	public XboxController controller = new XboxController(0);
	
	
	public final JoystickButton buttonA = new JoystickButton(controller, Constants.aButton);
	public final JoystickButton buttonB = new JoystickButton(controller, Constants.bButton);
	public final JoystickButton buttonX = new JoystickButton(controller, Constants.xButton);
	public final JoystickButton buttonY = new JoystickButton(controller, Constants.yButton);
	public final JoystickButton bumperL = new JoystickButton(controller, Constants.leftBumper);
	public final JoystickButton bumperR = new JoystickButton(controller, Constants.rightBumper);
	public final JoystickButton backB = new JoystickButton(controller, Constants.backButton);	
	public final JoystickButton startB = new JoystickButton(controller, Constants.startButton);
	public final JoystickButton leftStickB = new JoystickButton(controller, Constants.lStickButton);
	public final JoystickButton rightStickB = new JoystickButton(controller, Constants.rStickButton);

	//public final DigitalInput hatchSensor = new DigitalInput(1);
    
    private static RobotMap robotMap;
   

    private RobotMap ()
    {
		talonSetup(talonFL);
		talonSetup(talonBL);
		talonSetup(talonBR);
		talonSetup(talonFR);
		//talonSetup(talonBallIntake);
		talonSetup(talonBallShooter);

		ultrasonicSensor.setAutomaticMode(true);
    }
    
    public static RobotMap getRobotMap()
    {
        if (robotMap == null)
        {
            robotMap = new RobotMap();
        }

        return robotMap;
	}
	
	public void talonSetup(WPI_TalonSRX talon) //copied from last year
	{
		talon.configNominalOutputForward(0, Constants.timeOutMs);
		talon.configNominalOutputReverse(0, Constants.timeOutMs);
		talon.configPeakOutputForward(1, Constants.timeOutMs);
		talon.configPeakOutputReverse(-1, Constants.timeOutMs);
		talon.configAllowableClosedloopError(0, 0, Constants.timeOutMs);
		talon.configNeutralDeadband(0.05, Constants.timeOutMs); 
		talon.setNeutralMode(com.ctre.phoenix.motorcontrol.NeutralMode.Brake);
		talon.setInverted(false);

		// Peak current and duration must be exceeded before corrent limit is activated.
		// When activated, current will be limited to continuous current.
		// Set peak current params to 0 if desired behavior is to immediately
		// current-limit.
		talon.enableCurrentLimit(true);
		talon.configContinuousCurrentLimit(30, Constants.timeOutMs); // Must be 5 amps or more
		talon.configPeakCurrentLimit(30, Constants.timeOutMs); // 100 A
		talon.configPeakCurrentDuration(200, Constants.timeOutMs); // 200 ms
	}
	
	public double deadzone(double input)
	{
		if(Math.abs(input)>Constants.deadzone)
		{
			return input;
		}
		else
		{
			return 0;
		}
	}
	
	public double getLeftY()
	{
		return -deadzone(controller.getY(Hand.kLeft));
	}
	
	public double getLeftX()
	{
		return deadzone(controller.getX(Hand.kLeft));
	}
	
	public double getRightY()
	{
		return -deadzone(controller.getY(Hand.kRight));
	}
	
	public double getRightX()
	{
		return deadzone(controller.getX(Hand.kRight));
	}
	
	public double getTrigger()
	{
		if(controller.getTriggerAxis(Hand.kRight)>0)
		{
			return deadzone(controller.getTriggerAxis(Hand.kRight));
		}
		else if(controller.getTriggerAxis(Hand.kLeft)>0)
		{
			return -deadzone(controller.getTriggerAxis(Hand.kLeft));
		}
		else
		{
			return 0.0;
        }
    }

}
