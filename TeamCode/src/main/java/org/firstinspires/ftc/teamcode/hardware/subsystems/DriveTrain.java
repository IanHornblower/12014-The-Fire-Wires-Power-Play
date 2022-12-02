package org.firstinspires.ftc.teamcode.hardware.subsystems;

import static org.firstinspires.ftc.teamcode.hardware.DriveConstraints.TURN_WEIGHT;
import static org.firstinspires.ftc.teamcode.hardware.DriveConstraints.VX_WEIGHT;
import static org.firstinspires.ftc.teamcode.hardware.DriveConstraints.VY_WEIGHT;
import static org.firstinspires.ftc.teamcode.hardware.DriveConstraints.dt_trackWidth;
import static org.firstinspires.ftc.teamcode.hardware.DriveConstraints.maxAngularVelocity;
import static org.firstinspires.ftc.teamcode.hardware.DriveConstraints.maxVelocity;

import com.ThermalEquilibrium.homeostasis.Utils.Vector;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.apache.commons.math3.util.IterationListener;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.interfaces.Subsystem;
import org.firstinspires.ftc.teamcode.math.Pose2D;

@Config
public class DriveTrain implements Subsystem {
    DcMotorEx frontLeft, frontRight, backLeft, backRight;

    public static double trackWidth = 15;
    public static double wheelBase = 12;
    public static double lateralMultiplier = 0;

    double fl, fr, bl, br;

    public DcMotorEx[] motors;

    HardwareMap hwMap;

    //public Localizer localizer;

    public DriveTrain(Robot robot) {
        hwMap = robot.hwMap;
        frontLeft = hwMap.get(DcMotorEx.class, "fl"); // should be fl
        frontRight = hwMap.get(DcMotorEx.class, "fr");
        backLeft = hwMap.get(DcMotorEx.class, "bl");
        backRight = hwMap.get(DcMotorEx.class, "br");

        motors = new DcMotorEx[] {frontLeft, frontRight, backLeft, backRight};


        //localizer = new Localizer(robot);
    }

    public HardwareMap getHardwareMap() {
        return hwMap;
    }

    public void resetEncoders() {
        for(DcMotorEx motors:motors) {
            motors.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            motors.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        }
    }

    public void setWeightedDrivePower(double x, double y, double heading) {
        Pose2D vel = new Pose2D(x, y, heading);

        if (Math.abs(x) + Math.abs(y)
                + Math.abs(heading) > 1) {
            // re-normalize the powers according to the weights
            double denom = VX_WEIGHT * Math.abs(x)
                    + VY_WEIGHT * Math.abs(y)
                    + TURN_WEIGHT * Math.abs(heading);

            vel = new Pose2D(
                    VX_WEIGHT * x,
                    VY_WEIGHT * y,
                    TURN_WEIGHT * heading
            ).div(denom);
        }

        setMotorPowers(vel.x, vel.y, vel.heading);
    }

    public void setWeightedDrivePower(Pose2D pose) {
        setWeightedDrivePower(pose.x, pose.y, pose.heading);
    }

    public void setMotorPowers(double x, double y, double t) {
        double k = (dt_trackWidth + wheelBase) / 2.0;

        double power = Math.hypot(x, y) * Math.sqrt(2.0); // sqrt(2) == 2 * sin/cos(45)
        double angle = Math.atan2(y, x) - Math.PI / 4.0;

        double fl = power * Math.cos(angle) + Math.toRadians(k * t) * (maxAngularVelocity / maxVelocity);
        double bl = power * Math.sin(angle) + Math.toRadians(k * t) * (maxAngularVelocity / maxVelocity);
        double fr = power * Math.sin(angle) - Math.toRadians(k * t) * (maxAngularVelocity / maxVelocity);
        double br = power * Math.cos(angle) - Math.toRadians(k * t) * (maxAngularVelocity / maxVelocity);

        setMotorPowers(fl, fr, bl, br);
    }

    public void setMotorPowers(double fl, double fr, double bl, double br) {
        this.fl = fl;
        this.fr = fr;
        this.bl = bl;
        this.br = br;
    }

    public void setMotorPowers(Vector vector) {
        setMotorPowers(vector.get(0), vector.get(1));
    }

    public void setMotorPowers(double left, double right) {
        setMotorPowers(left, right, left, right);
    }

    public void stopDriveTrain() {
        setMotorPowers(0, 0, 0, 0);
    }

    @Override
    public void init() throws InterruptedException {
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);

        for(DcMotorEx m : motors) {
            m.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        resetEncoders();
    }

    @Override
    public void update() throws InterruptedException {
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);

       // localizer.update();
    }
}
