package org.firstinspires.ftc.teamcode.ActionSystem.actions;

import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.ActionSystem.Action;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.ConeManipulator;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Intake;

public class V4BSetPosition extends Action {

    Robot robot;
    double position;

    public double tolerance = 50;

    boolean instant = false;

    public V4BSetPosition(Robot robot, double position) {
        this.robot = robot;
        this.position = position;
    }

    public V4BSetPosition(Robot robot, double position, double tolerance) {
        this.robot = robot;
        this.position = position;
        this.tolerance = tolerance;
    }

    public V4BSetPosition(Robot robot, ConeManipulator.V4BPreset pos) {
        this.robot = robot;
        if(robot.intake.direction == Intake.DIRECTION.front) {
            position = (pos.getFront());
        }
        else {
            position = (pos.getBack());
        }
    }

    public V4BSetPosition(Robot robot, ConeManipulator.V4BPreset pos, double tolerance) {
        this.robot = robot;
        if(robot.intake.direction == Intake.DIRECTION.front) {
            position = (pos.getFront());
        }
        else {
            position = (pos.getBack());
        }
        this.tolerance = tolerance;
    }

    public V4BSetPosition(Robot robot, double position, boolean instant) {
        this.robot = robot;
        this.position = position;
        this.instant = instant;
    }

    public V4BSetPosition(Robot robot, double position, double tolerance, boolean instant) {
        this.robot = robot;
        this.position = position;
        this.tolerance = tolerance;
        this.instant = instant;
    }

    public V4BSetPosition(Robot robot, ConeManipulator.V4BPreset pos, boolean instant) {
        this.robot = robot;
        if(robot.intake.direction == Intake.DIRECTION.front) {
            position = (pos.getFront());
        }
        else {
            position = (pos.getBack());
        }
        this.instant = instant;
    }

    public V4BSetPosition(Robot robot, ConeManipulator.V4BPreset pos, double tolerance, boolean instant) {
        this.robot = robot;
        this.instant = instant;
        if(robot.intake.direction == Intake.DIRECTION.front) {
            position = (pos.getFront());
        }
        else {
            position = (pos.getBack());
        }
        this.tolerance = tolerance;
    }

    @Override
    public void startAction() {
        robot.coneManipulator.setPosition(position);
    }

    @Override
    public void runAction() throws InterruptedException {
        error = position - robot.coneManipulator.fourbar.getCurrentPosition();
        isComplete = Math.abs(error) < tolerance || instant;
    }

    @Override
    public void stopAction() {
    }
}
