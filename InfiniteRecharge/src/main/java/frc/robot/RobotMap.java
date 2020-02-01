/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;




import com.ctre.phoenix.motorcontrol.can.WPI_TalonSRX;
import edu.wpi.first.wpilibj.*;
/**
 * Add your docs here.
 */
public class RobotMap {
	public final WPI_TalonSRX talonFL =  new WPI_TalonSRX (4); //4
    public final WPI_TalonSRX talonFR =  new WPI_TalonSRX (9); //3
    public final WPI_TalonSRX talonBL =  new WPI_TalonSRX (2); //2
	public final WPI_TalonSRX talonBR =  new WPI_TalonSRX (1); //1
	public final WPI_TalonSRX talonBallIntake = new WPI_TalonSRX(5);
	public final WPI_TalonSRX talonBallShooter = new WPI_TalonSRX(6); //6

	public XboxController controller = new XboxController(0);

	private RobotMap() {
		
	}
	
}