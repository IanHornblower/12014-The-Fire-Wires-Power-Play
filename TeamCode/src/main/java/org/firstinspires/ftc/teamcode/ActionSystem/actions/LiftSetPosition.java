package org.firstinspires.ftc.teamcode.ActionSystem.actions;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.ActionSystem.Action;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Lift;

@Config
public class LiftSetPosition extends Action {

    Robot robot;
    double position;

    public static double tolerance = 10;
    public static double zeroMovementPower = 0.05;

    double kP = Lift.kP;

    double error = 1e+9;

    public LiftSetPosition(Robot robot, double position) {
        this.robot = robot;
        this.position = position;
    }

    @Override
    public void startAction() {

    }

    @Override
    public void runAction() throws InterruptedException {
        error = position - robot.lift.getEncoderPosition();

        robot.lift.setPower(error * kP);

        isComplete = Math.abs(error) < tolerance;
        isComplete = Math.abs(error * kP) < zeroMovementPower;
        //isComplete = Math.abs(error) < tolerance || robot.lift.limitSwitchPressed();
    }

    @Override
    public void stopAction() {
        robot.lift.setPower(0);
    }
}
