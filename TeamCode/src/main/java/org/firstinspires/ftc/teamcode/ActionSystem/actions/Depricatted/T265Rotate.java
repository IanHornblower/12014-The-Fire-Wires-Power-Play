package org.firstinspires.ftc.teamcode.ActionSystem.actions.Depricatted;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.AngleController;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.ActionSystem.Action;
import org.firstinspires.ftc.teamcode.hardware.DriveConstraints;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.math.Curve;
import org.firstinspires.ftc.teamcode.math.Pose2D;

public class T265Rotate extends Action {

    DriveTrain dt;
    Robot robot;
    double heading;

    BasicPID headingPID;
    AngleController headingController;

    public T265Rotate(Robot robot, double heading) {
        this.dt = robot.driveTrain;
        this.robot = robot;
        this.heading = heading;

        headingPID = new BasicPID(DriveConstraints.inplace_headingPID);
        headingController = new AngleController(headingPID);
    }

    @Override
    public void startAction() {

    }

    @Override
    public void runAction() throws InterruptedException {
        error = angleWrap(heading - robot.imu.getHeadingInRadians());
        //double headingP = headingController.calculate(AngleUnit.normalizeRadians(heading - pos.heading), 0);
        dt.setMotorPowers(error * DriveConstraints.inplace_headingPID.Kp, -error * DriveConstraints.inplace_headingPID.Kp);

        isComplete = error < Math.toRadians(3);
    }

    private double angleWrap(double radians) {

        while (radians > Math.PI) {
            radians -= 2 * Math.PI;
        }
        while (radians < -Math.PI) {
            radians += 2 * Math.PI;
        }

        return radians;
    }

    @Override
    public void stopAction() {
        dt.stopDriveTrain();
    }



}