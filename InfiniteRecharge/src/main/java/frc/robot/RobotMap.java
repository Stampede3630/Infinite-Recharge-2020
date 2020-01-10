package frc.robot;

public class RobotMap {
    public static final int DRIVETRAIN_FRONT_LEFT_ANGLE_MOTOR = 0; // CAN
    public static final int DRIVETRAIN_FRONT_LEFT_ANGLE_ENCODER = 3; // Analog
    public static final int DRIVETRAIN_FRONT_LEFT_DRIVE_MOTOR = 1; // CAN
    public static final double FRONT_LEFT_ANGLE_OFFSET = 1.969 + (Math.PI/2);

    public static final int DRIVETRAIN_FRONT_RIGHT_ANGLE_MOTOR = 6; // CAN
    public static final int DRIVETRAIN_FRONT_RIGHT_ANGLE_ENCODER = 0; // Analog
    public static final int DRIVETRAIN_FRONT_RIGHT_DRIVE_MOTOR = 3; // CAN
    public static final double FRONT_RIGHT_ANGLE_OFFSET = 0.390 - (Math.PI / 4) - 0.27 + .2 - (Math.PI/2);

    public static final int DRIVETRAIN_BACK_LEFT_ANGLE_MOTOR = 4; // CAN
    public static final int DRIVETRAIN_BACK_LEFT_ANGLE_ENCODER = 1; // Analog
    public static final int DRIVETRAIN_BACK_LEFT_DRIVE_MOTOR = 5; // CAN
    public static final double BACK_LEFT_ANGLE_OFFSET = 0.533 - 1.06 - (Math.PI/2);

    public static final int DRIVETRAIN_BACK_RIGHT_ANGLE_MOTOR = 2; // CAN
    public static final int DRIVETRAIN_BACK_RIGHT_ANGLE_ENCODER = 2; // Analog
    public static final int DRIVETRAIN_BACK_RIGHT_DRIVE_MOTOR = 7; // CAN
    public static final double BACK_RIGHT_ANGLE_OFFSET = 2.5 + 3.1 - (Math.PI/2);

}