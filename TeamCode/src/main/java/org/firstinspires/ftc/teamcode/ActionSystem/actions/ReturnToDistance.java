package org.firstinspires.ftc.teamcode.ActionSystem.actions;

import org.firstinspires.ftc.teamcode.ActionSystem.Action;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.LineAlignment;

public class ReturnToDistance extends Action {

    Robot robot;
    double kP;

    public ReturnToDistance(Robot robot, double kP) {
     this.robot = robot;
     this.kP = kP;
    }

    @Override
    public void startAction() {

    }

    @Override
    public void runAction() throws InterruptedException {
        error = LineAlignment.wallTolerance - robot.lineAlignment.getDistance();
        robot.driveTrain.setMotorPowers(0, error * 0.0015, 0);

        isComplete =
                LineAlignment.wallTolerance-10 < robot.lineAlignment.getDistance() && robot.lineAlignment.getDistance() < LineAlignment.wallTolerance+10 && robot.driveTrain.motors[0].getVelocity() < 20;
    }

    @Override
    public void stopAction() {
        robot.driveTrain.stopDriveTrain();
    }
}
