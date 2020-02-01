/*----------------------------------------------------------------------------*/
/* Copyright (c) 2018-2019 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package frc.robot;

import edu.wpi.first.wpilibj.controller.PIDController;
import edu.wpi.first.wpilibj.controller.ProfiledPIDController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.trajectory.TrapezoidProfile;

/**
 * Add your docs here.
 */
public class BallFollowDrive {

    private static ProfiledPIDController turnPID = new ProfiledPIDController(0.03, 0, 0,
            new TrapezoidProfile.Constraints(Robot.kMaxAngularSpeed, Math.PI * 6));
    private static PIDController xPID = new PIDController(0.03, 0, 0);
    private static PIDController yPID = new PIDController(0.03, 0, 0);
    private static PIDController xVelPID = new PIDController(0.03, 0, 0);
    private static PIDController yVelPID = new PIDController(0.03, 0, 0);

    private static TargetPos targetPos = new TargetPos();

    private enum IntakeState {
        Searching, Intaking, Done
    }

    private static IntakeState intakeState;

    private static double lastYAngle;

    private static int lastAngleInvalidTicks;

    public static void initLimelight() {
        LimeLight.enableVisionProcessing();
        LimeLight.setPipeline(RobotMap.PIPELINE_BALL_FOLLOW);
    }

    public static void reset() {

        resetIntakeState();
    }

    public static void resetIntakeState() {
        intakeState = IntakeState.Searching;
    }

    public static boolean intake(double rotationSpeed) {
        targetPos.getPosFromLimelight();
        double xTargetAngle = LimeLight.getTX();

        // System.out.println(xScreenPos);

        boolean hasTarget = targetPos.isValid();
        boolean isInCenter = Math.abs(xTargetAngle) < 16;

        switch (intakeState) {
        case Searching:
            SmartDashboard.putNumber("Intake State", 0);
            break;
        case Intaking:
            SmartDashboard.putNumber("Intake State", 1);
            break;
        case Done:
            SmartDashboard.putNumber("Intake State", 2);
            break;
        }

        SmartDashboard.putNumber("Last Y Angle", lastYAngle);

        if (lastYAngle < -20) {
            lastAngleInvalidTicks = 0;
        } else {
            lastAngleInvalidTicks++;
        }

        switch (intakeState) {
        case Searching:
            if (hasTarget) {
                if (isInCenter) {
                    intakeState = IntakeState.Intaking;
                    stop();
                    lastYAngle = LimeLight.getTY();
                    break;
                } else {
                    Drivetrain.drive(0, 0, Robot.kMaxAngularSpeed * 0.6, false);
                }
            } else {
                Drivetrain.drive(0, 0, Robot.kMaxAngularSpeed * 0.9, false);
            }
            break;
        case Intaking:
            if (hasTarget) {
                lastYAngle = LimeLight.getTY();
                // stop();
                followTarget(targetPos, 0, 0);
            } else {
                if (lastAngleInvalidTicks < 4) // The ball was last seen in the lower quarter of the screen
                {
                    intakeState = IntakeState.Done;
                } else {
                    intakeState = IntakeState.Searching;
                }
                stop();
            }
            break;
        case Done:
            stop();
            break;
        }

        return intakeState == IntakeState.Done;
    }

    public static void stop() {
        Drivetrain.stop();
    }

    public static void followTarget(TargetPos targetPos, double targetX, double targetY) {
        if (!targetPos.isValid()) {
            Drivetrain.stop();
            return;
        }
        double xMotion = 0;
        double yMotion = 0;

        double xDelta = targetPos.getX() - targetX;
        double yDelta = targetPos.getY() - targetY;

        xMotion = -xPID.calculate(xDelta, 0);
        yMotion = yPID.calculate(yDelta, 0);

        // final double POWER = 1;
        //
        // xMotion = Math.pow(Math.abs(xMotion), POWER) * Math.signum(xMotion);
        // yMotion = Math.pow(Math.abs(yMotion), POWER) * Math.signum(yMotion);

        double dist = Math.sqrt(Math.pow(xDelta, 2) + Math.pow(yDelta, 2));

        final double DIST_THRESHOLD = 5;

        if (dist < DIST_THRESHOLD) {
            xMotion *= Math.sqrt((DIST_THRESHOLD - dist) / DIST_THRESHOLD);
            yMotion *= Math.sqrt((DIST_THRESHOLD - dist) / DIST_THRESHOLD);
        }

        final double SPEEDMULT = 0.7;

        xMotion *= -SPEEDMULT;
        yMotion *= -SPEEDMULT;

        final double DELTA_MULT = 0.5;

        double xVel = xVelPID.calculate(targetPos.getXVel()) * -DELTA_MULT;
        double yVel = yVelPID.calculate(targetPos.getYVel()) * -DELTA_MULT;

        xVel = Math.pow(Math.abs(xVel), 2) * Math.signum(xVel);
        yVel = Math.pow(Math.abs(yVel), 2) * Math.signum(yVel);

        xMotion -= xVel;
        xMotion -= yVel;

        SmartDashboard.putNumber("dist", dist);

        double angleDelta = Math.atan2(xDelta, yDelta) / Math.PI * 180;
        // System.out.println(angleDelta);
        double angleVel = turnPID.calculate(angleDelta, 0);

        // Drivetrain.drive(-MathHelper.remap(xMotion, -1, 1, -Robot.kMaxSpeed,
        // Robot.kMaxSpeed) / 4,
        // MathHelper.remap(yMotion, -1, 1, -Robot.kMaxSpeed, Robot.kMaxSpeed) / 4,
        // MathHelper.remap(angleVel, -1, 1, -Robot.kMaxAngularSpeed,
        // Robot.kMaxAngularSpeed) / 2, false);
        Drivetrain.drive(MathHelper.clampUnit(yMotion) * Robot.kMaxSpeed / 2,
                MathHelper.clampUnit(xMotion) * Robot.kMaxSpeed / 2,
                MathHelper.clampUnit(angleVel) * Robot.kMaxAngularSpeed, false);

        SmartDashboard.putNumber("X Delta", xDelta);
        SmartDashboard.putNumber("Y Delta", yDelta);
        SmartDashboard.putNumber("Angle Delta", angleDelta);
        SmartDashboard.putNumber("X Velocity", xMotion);
        SmartDashboard.putNumber("Y Velocity", yMotion);
        SmartDashboard.putNumber("Angular Velocity", angleVel);
    }

}