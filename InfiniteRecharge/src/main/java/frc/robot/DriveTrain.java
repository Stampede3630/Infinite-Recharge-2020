/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

<<<<<<< HEAD
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Translation2d;
import edu.wpi.first.wpilibj.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.kinematics.SwerveDriveOdometry;
import edu.wpi.first.wpilibj.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.kauailabs.navx.frc.AHRS;
public class Drivetrain {

    Drivetrain drivetrain;

    public Drivetrain() {
      

    }

    public void drive(){
      
    }

    public static Drivetrain getInstance(){
        if(drivetrain == null){
            drivetrain = new Drivetrain();
        }
        return drivetrain;
    }
  
   
=======
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.SpeedControllerGroup;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

public class DriveTrain {
	private static final SpeedControllerGroup leftMotors = new SpeedControllerGroup(RobotMap.TALON_FL,
			RobotMap.TALON_BL);
	private static final SpeedControllerGroup rightMotors = new SpeedControllerGroup(RobotMap.TALON_FR,
			RobotMap.TALON_BR);
	private static final DifferentialDrive differentialDrive = new DifferentialDrive(leftMotors, rightMotors);

	public static void drive()
	{
		double speed = RobotMap.controller.getY(Hand.kLeft);
		double rotation = RobotMap.controller.getX(Hand.kLeft);

		if(Math.abs(RobotMap.controller.getY(Hand.kLeft))<0.1){
			speed = 0;
		}
		if(Math.abs(RobotMap.controller.getX(Hand.kLeft))<0.1){
			rotation = 0;
		}
		drive(speed, rotation);
	}

	public static void drive(double xSpeed, double zRotation)
	{

		differentialDrive.arcadeDrive(xSpeed, zRotation, true);
	}

	public static void stop()
	{
		drive(0, 0);
	}
}
>>>>>>> d6431b9f10f6bbf78dec9e5c8c1df2ff4c323960
