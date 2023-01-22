package org.firstinspires.ftc.teamcode.ActionSystem.actions;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.ActionSystem.Action;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Lift;

public class LiftSetPosition extends Action {

    Robot robot;
    double position;

    public double tolerance = 100;
    double error = 1e+9;

    public LiftSetPosition(Robot robot, double position) {
        this.robot = robot;
        this.position = position;
    }

    public LiftSetPosition(Robot robot, double position, double tolerance) {
        this.robot = robot;
        this.position = position;
        this.tolerance = tolerance;
    }

    @Override
    public void startAction() {
        robot.lift.setPosition(position);
    }

    @Override
    public void runAction() throws InterruptedException {
        error = position - robot.lift.getEncoderPosition();

        isComplete = error < tolerance;
    }

    @Override
    public void stopAction() {

    }
}
