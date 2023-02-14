package org.firstinspires.ftc.teamcode.ActionSystem.actions;

import com.acmerobotics.dashboard.config.Config;

import org.firstinspires.ftc.teamcode.ActionSystem.Action;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Lift;

public class LiftSetPosition extends Action {

    Robot robot;
    double position;

    public double tolerance = 30;
    boolean instant = false;

    public LiftSetPosition(Robot robot, double position) {
        this.robot = robot;
        this.position = position;
    }

    public LiftSetPosition(Robot robot, double position, double tolerance) {
        this.robot = robot;
        this.position = position;
        this.tolerance = tolerance;
    }

    public LiftSetPosition(Robot robot, double position, boolean instant) {
        this.robot = robot;
        this.position = position;
        this.instant = instant;
    }

    public LiftSetPosition(Robot robot, double position, double tolerance, boolean instant) {
        this.robot = robot;
        this.position = position;
        this.tolerance = tolerance;
        this.instant = instant;
    }

    @Override
    public void startAction() {
        robot.lift.setPosition(position);
    }

    @Override
    public void runAction() throws InterruptedException {
        error = position - robot.lift.getEncoderPosition();
        isComplete = Math.abs(error) < tolerance || instant;
    }

    @Override
    public void stopAction() {

    }
}
