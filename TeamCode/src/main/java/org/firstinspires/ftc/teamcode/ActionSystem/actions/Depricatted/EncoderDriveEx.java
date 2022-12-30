package org.firstinspires.ftc.teamcode.ActionSystem.actions.Depricatted;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.AngleController;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;

import org.firstinspires.ftc.teamcode.ActionSystem.Action;
import org.firstinspires.ftc.teamcode.hardware.DriveConstraints;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.DriveTrain;

public class EncoderDriveEx extends Action {

    Robot robot;
    DriveTrain dt;
    double x, y, heading;
    int ticks;

    double[] totalTicks = new double[4];

    boolean fieldCentric;

    BasicPID headingPID = new BasicPID(new PIDCoefficients(
            DriveConstraints.EncoderDriveHeadingPID[0],
            DriveConstraints.EncoderDriveHeadingPID[1],
            DriveConstraints.EncoderDriveHeadingPID[2])
    );
    AngleController headingController = new AngleController(headingPID);

    public EncoderDriveEx(Robot robot, double x, double y, double heading, int ticks) {
        this.robot = robot;
        this.dt = robot.driveTrain;
        this.ticks = ticks;
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public EncoderDriveEx(Robot robot, double x, double y, double heading, int ticks, boolean fieldCentric) {
        this.robot = robot;
        this.dt = robot.driveTrain;
        this.ticks = ticks;
        this.x = x;
        this.y = y;
        this.heading = heading;
        this.fieldCentric = fieldCentric;
    }

    @Override
    public void startAction() {
    }

    @Override
    public void runAction() throws InterruptedException {
        if(fieldCentric) {
            dt.driveFieldCentric(x, y, headingController.calculate(heading, robot.imu.getHeadingInRadians()));
        }
        else {
            dt.setMotorPowers(x, y, headingController.calculate(heading, robot.imu.getHeadingInRadians()));
        }

        for(int i = 0; i < 4; i++) {
            totalTicks[i] += Math.abs(dt.motors[i].getCurrentPosition())/(double)DriveConstraints.encoderResolution;
            isComplete =  totalTicks[i] > ticks;
        }
    }

    @Override
    public void stopAction() {
        dt.stopDriveTrain();
    }
}