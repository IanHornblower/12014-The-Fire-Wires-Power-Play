package org.firstinspires.ftc.teamcode.ActionSystem.actions.Depricatted;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.AngleController;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;

import org.firstinspires.ftc.teamcode.ActionSystem.Action;
import org.firstinspires.ftc.teamcode.hardware.DriveConstraints;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.math.Curve;
import org.firstinspires.ftc.teamcode.math.Pose2D;

public class RunToPosition extends Action {

    Robot robot;
    DriveTrain dt;
    Pose2D endPos;

    BasicPID xPID, yPID, headingPID;
    AngleController headingController;

    public RunToPosition(Robot robot, Pose2D endPos) {
        this.robot = robot;
        this.dt = robot.driveTrain;
        this.endPos = endPos;

        xPID = new BasicPID(DriveConstraints.xPID);
        yPID = new BasicPID(DriveConstraints.yPID);
        headingPID = new BasicPID(DriveConstraints.headingPID);

        headingController = new AngleController(headingPID);
    }

    @Override
    public void startAction() {
    }

    @Override
    public void runAction() throws InterruptedException {
        Pose2D pos = robot.t265.getPose();

        double xP = xPID.calculate(endPos.x, pos.x);
        double yP = yPID.calculate(endPos.y, pos.y);
        double headingP = headingController.calculate(endPos.heading, pos.heading);

        dt.driveFieldCentric(xP, yP, headingP);

        //if(error < 2 && Curve.getShortestDistance(endPos.heading, pos.heading) > Math.toRadians(3)) {
        //    dt.driveFieldCentric(0, 0, headingP);
        //}

        error = Math.abs(endPos.getDistanceFrom(pos.toPoint()));
        isComplete = error < 2;
    }

    @Override
    public void stopAction() {
        dt.stopDriveTrain();
    }
}