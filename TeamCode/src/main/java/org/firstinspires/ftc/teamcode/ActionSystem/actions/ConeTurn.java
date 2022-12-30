package org.firstinspires.ftc.teamcode.ActionSystem.actions;

import org.firstinspires.ftc.teamcode.ActionSystem.Action;
import org.firstinspires.ftc.teamcode.control.AsymStuff.ConeTurnOnlyControl;
import org.firstinspires.ftc.teamcode.control.AsymStuff.TurnOnlyControl;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.vision.CombinedTracker;

public class ConeTurn extends Action {

    Robot robot;
    ConeTurnOnlyControl controller;

    public ConeTurn(Robot robot) {
        this.robot = robot;
        this.controller = new ConeTurnOnlyControl(()-> robot.rearCamera.combinedTracker.getConeError());
    }

    @Override
    public void startAction() {
        robot.rearCamera.combinedTracker.setTrackType(CombinedTracker.TrackType.CONE);
    }

    @Override
    public void runAction() {
        robot.driveTrain.setMotorPowers(controller.calculate().scalarMultiply(1));
        isComplete = Math.abs(controller.getEndGoalError()) < 10; // 10 is in pixels
                //&& Math.abs(robot.imu.getAccel()) < Math.toRadians(10);

    }

    @Override
    public void stopAction() {
        robot.driveTrain.setMotorPowers(0,0);
    }
}