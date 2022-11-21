package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.interfaces.Subsystem;

@Config
public class Lift implements Subsystem {

    public DcMotorEx lower, upper;
    HardwareMap hwMap;

    public static double lowest = 0;
    public static double top = 100;

    double lower_p = 0.0, upper_p = 0.0;

    public static double feedfoward = 0.0;


    public Lift(Robot robot) {
        hwMap = robot.hwMap;

        lower = hwMap.get(DcMotorEx.class, "lowerLift");
        upper = hwMap.get(DcMotorEx.class, "upperLift");
    }

    public void controlLift(double power) {
        double ff = getEncoderPosition() * feedfoward;

        setPower(ff + power);
    }

    public void setPosition(double power, double position, double tolerance) {
        double multiplier = Math.signum(position - getEncoderPosition());



        setPower(power * multiplier);
    }

    public double getEncoderPosition() {
        return lower.getCurrentPosition();
    }

    public void setPower(double power) {
        lower.setPower(power);
        upper.setPower(power);
    }

    @Override
    public void init() throws InterruptedException {
        upper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lower.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }

    @Override
    public void update() throws InterruptedException {
    }


}
