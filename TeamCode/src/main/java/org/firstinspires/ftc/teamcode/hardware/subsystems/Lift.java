package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.interfaces.Subsystem;

@Config
public class Lift implements Subsystem {

    public DcMotorEx lower, upper;
    HardwareMap hwMap;
    public DigitalChannel limitSwitch;

    public static double lowest = 0;
    public static double lowResting = 25;
    public static double smallPole = 500;
    public static double middlePole = 1350;
    public static double highPole = 1950;
    public static double top = 2600;

    public static double kP = 0.013;

    public static double feedfoward = 0.0;


    public Lift(Robot robot) {
        hwMap = robot.hwMap;

        lower = hwMap.get(DcMotorEx.class, "lowerLift");
        upper = hwMap.get(DcMotorEx.class, "upperLift");

        limitSwitch = hwMap.get(DigitalChannel.class, "lm");
    }


    public void setPosition(double power, double position, double tolerance) {
        double multiplier = Math.signum(position - getEncoderPosition());

        setPower(power * multiplier);
    }

    public double getEncoderPosition() {
        return lower.getCurrentPosition();
    }

    public void runToPosition(double position) {
        double error = position - getEncoderPosition();

        setPower(error * kP);
    }

    public boolean isLiftDown() {
        return !limitSwitch.getState();
    }

    public void setPower(double power) {
        lower.setPower(power);
        upper.setPower(power);
    }

    @Override
    public void init() throws InterruptedException {
        upper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lower.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        upper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lower.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        upper.setDirection(DcMotorSimple.Direction.REVERSE);
        lower.setDirection(DcMotorSimple.Direction.REVERSE);

        upper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        lower.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        limitSwitch.setMode(DigitalChannel.Mode.INPUT);
    }

    @Override
    public void update() throws InterruptedException {
    }


}
