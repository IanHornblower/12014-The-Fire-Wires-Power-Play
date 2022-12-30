package org.firstinspires.ftc.teamcode.ActionSystem.actions;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.AngleController;

import org.firstinspires.ftc.teamcode.ActionSystem.Action;
import org.firstinspires.ftc.teamcode.control.AsymStuff.TurnOnlyControl;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.LineAlignment;

public class FollowBlueLineToDistance extends Action {

    Robot robot;
    double kP;
    double heading;

    TurnOnlyControl turnOnlyControl;

    public FollowBlueLineToDistance(Robot robot, double kP, double heading) {
     this.robot = robot;
     this.kP = kP;
     this.heading = heading;
     this.turnOnlyControl = new TurnOnlyControl(()-> robot.imu.getHeadingInRadians(), heading);
    }

    @Override
    public void startAction() {

    }

    @Override
    public void runAction() throws InterruptedException {

        if(robot.lineAlignment.getLeft()) {
            // Align and rotate
            robot.driveTrain.setMotorPowers(0.2, error * kP, turnOnlyControl.calculate().get(0));
        }

        if(robot.lineAlignment.getRight()) {
            // Align and rotate
            robot.driveTrain.setMotorPowers(-0.2, error * kP, turnOnlyControl.calculate().get(0));
        }

        error = LineAlignment.wallTolerance - robot.lineAlignment.getDistance();
        robot.driveTrain.setMotorPowers(0, error * kP, 0);

        if(LineAlignment.wallTolerance-10 < robot.lineAlignment.getDistance() && // If at the end position but not at the desired angle rotate to it
                robot.lineAlignment.getDistance() < LineAlignment.wallTolerance+10 &&
                robot.driveTrain.motors[0].getVelocity() < 20 &&
                turnOnlyControl.getEndGoalError() > Math.toRadians(2)) {
            robot.driveTrain.setMotorPowers(turnOnlyControl.calculate());
        }

        isComplete = LineAlignment.wallTolerance-10 < robot.lineAlignment.getDistance() && // If everything is good finish
                robot.lineAlignment.getDistance() < LineAlignment.wallTolerance+10 &&
                robot.driveTrain.motors[0].getVelocity() < 20 &&
                turnOnlyControl.getEndGoalError() < Math.toRadians(2);
    }

    @Override
    public void stopAction() {
        robot.driveTrain.stopDriveTrain();
    }
}
