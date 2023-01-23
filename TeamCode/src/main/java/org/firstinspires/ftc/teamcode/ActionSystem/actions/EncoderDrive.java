package org.firstinspires.ftc.teamcode.ActionSystem.actions;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.AngleController;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.teamcode.ActionSystem.Action;
import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.DriveConstraints;
import org.firstinspires.ftc.teamcode.hardware.Robot;

public class EncoderDrive extends Action {

    Robot robot;
    SampleMecanumDrive dt;
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
        robot.driveTrain.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        robot.driveTrain.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void runAction() {
        double heading_error = headingController.calculate(heading, robot.imu.getHeadingInRadians());

        if(fieldCentric) {
          //  dt.dr(x, y, heading_error);
        }
        else {
          //  dt.setMotorPowers(x, y, heading_error);
        }

        for(double d : dt.getWheelPositions()) {
            isComplete = Math.abs(d) > ticks;
        }
    }

    @Override
    public void stopAction() {
        dt.setMotorPowers(0, 0, 0,0);
    }
}
