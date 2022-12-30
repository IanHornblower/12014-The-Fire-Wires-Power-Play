package org.firstinspires.ftc.teamcode.ActionSystem.actions.Depricatted;


import android.os.Build;

import androidx.annotation.RequiresApi;

import com.ThermalEquilibrium.homeostasis.Utils.Vector;

import org.firstinspires.ftc.teamcode.ActionSystem.Action;
import org.firstinspires.ftc.teamcode.control.AsymStuff.GoToGoalController;
import org.firstinspires.ftc.teamcode.hardware.Robot;

public class GoToPosition extends Action {

    GoToGoalController controller;
    Robot robot;
    Vector targetPose;
    GoToGoalController.driveDirection direction = GoToGoalController.driveDirection.FORWARD;

    public GoToPosition(Robot robot, Vector targetPose) {
        this.robot = robot;
        this.targetPose = targetPose;
    }

    public GoToPosition(Robot robot, Vector targetPose, GoToGoalController.driveDirection direction) {
        this.robot = robot;
        this.targetPose = targetPose;
        this.direction = direction;
    }


    @Override
    public void startAction() {
        this.controller = new GoToGoalController(robot, targetPose, direction);
    }

    @Override
    public void runAction() {
        try {
            isComplete = controller.update();
        } catch (Exception e) {
            e.printStackTrace();
        }
    }

    @Override
    public void stopAction() {
        robot.driveTrain.setMotorPowers(0, 0);
    }

}
