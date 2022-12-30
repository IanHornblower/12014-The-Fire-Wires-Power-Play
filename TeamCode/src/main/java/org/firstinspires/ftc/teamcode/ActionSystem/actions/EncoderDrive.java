package org.firstinspires.ftc.teamcode.ActionSystem.actions;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.AngleController;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.ActionSystem.Action;
import org.firstinspires.ftc.teamcode.hardware.DriveConstraints;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.DriveTrain;

public class EncoderDrive extends Action {

    Robot robot;
    DriveTrain dt;
    double x, y, heading;
    int ticks;

    boolean fieldCentric;

    BasicPID headingPID = new BasicPID(new PIDCoefficients(
            DriveConstraints.EncoderDriveHeadingPID[0],
            DriveConstraints.EncoderDriveHeadingPID[1],
            DriveConstraints.EncoderDriveHeadingPID[2])
    );
    AngleController headingController = new AngleController(headingPID);

    public EncoderDrive(Robot robot, double x, double y, double heading, int ticks) {
        this.robot = robot;
        this.dt = robot.driveTrain;
        this.ticks = ticks;
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public EncoderDrive(Robot robot, double angle, double speed, double heading, int ticks, boolean fieldCentric, boolean angleDrive) {
        this.robot = robot;
        this.dt = robot.driveTrain;
        this.ticks = ticks;
        this.x = Math.cos(angle) * speed;
        this.y = Math.sin(angle) * speed;
        this.heading = heading;
        this.fieldCentric = fieldCentric;
    }

    public EncoderDrive(Robot robot, double x, double y, double heading, int ticks, boolean fieldCentric) {
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
        robot.driveTrain.resetEncoders();
    }

    @Override
    public void runAction() {
        double heading_error = headingController.calculate(heading, robot.imu.getHeadingInRadians());

        if(fieldCentric) {
            dt.driveFieldCentric(x, y, heading_error);
        }
        else {
            dt.setMotorPowers(x, y, heading_error);
        }

        for(DcMotorEx motor : dt.motors) {
            isComplete = Math.abs(motor.getCurrentPosition()) > ticks;
        }
    }

    @Override
    public void stopAction() {
        dt.stopDriveTrain();
    }
}
