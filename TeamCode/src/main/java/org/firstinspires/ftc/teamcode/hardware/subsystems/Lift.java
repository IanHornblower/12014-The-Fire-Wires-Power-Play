package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.interfaces.Subsystem;
import org.firstinspires.ftc.teamcode.util.BasicPIDFixed;
import org.firstinspires.ftc.teamcode.util.MotionConstraint;
import org.firstinspires.ftc.teamcode.util.ProfiledPID;

@Config
public class Lift implements Subsystem {

    public DcMotorEx lower, middle, upper;
    HardwareMap hwMap;

    public static double Kp = 0.007;
    public static double Ki = 0.0002;
    public static double Kd = 0.0000;
    public static double Kg = 0.07;

    PIDCoefficients coef = new PIDCoefficients(Kp, Ki, Kd);
    BasicPID liftPID = new BasicPID(coef);

    MotionConstraint up = new MotionConstraint(0,0, 0);
    MotionConstraint down = new MotionConstraint(0, 0,0);
    ProfiledPID controlLoop = new ProfiledPID(up, down, coef);

    public double position = 0;
    public static double lowest = 0;
    public static double smallPole = 150;
    public static double middlePole = 560;
    public static double highPole = 980;
    public static double highPoleBroken = 1220;
    public static double top = 1300;

    public static double cone2 = 110;
    public static double cone3 = 150;
    public static double cone4 = 200;
    public static double cone5 = 240;

    public double error;

    public enum LIFT {
        RETURN(0),
        LOW(smallPole),
        MID(middlePole),
        HIGH(highPole),
        SUPERHIGH(highPoleBroken);

        double ticks;

        LIFT(double ticks) {
            this.ticks = ticks;
        }

        public double getTicks() {
            return ticks;
        }
    }

    public boolean manueal = false;

    public void setModeManuel() {
        manueal = true;
    }

    public void setModeAutomatic() {
        manueal = false;
    }

    public void regeneratePID() {
        coef = new PIDCoefficients(Kp, Ki, Kd);
        liftPID = new BasicPID(coef);
        controlLoop = new ProfiledPID(up, down, coef);
    }

    public Lift(Robot robot) {
        hwMap = robot.hwMap;

        lower = hwMap.get(DcMotorEx.class, "lowerLift");
        middle = hwMap.get(DcMotorEx.class, "middleLift");
        upper = hwMap.get(DcMotorEx.class, "upperLift");
    }

    public static double tolerance = 30;

    public void runToPosition(double position) {
        error = position - getEncoderPosition();
        if(error < 0 && Math.abs(error) > tolerance) {
            setPower(-0.2);
        }
        else if(error > 0 && Math.abs(error) > tolerance) {
            setPower(1);
        }
        else {
            setPower(Kg);
        }//
    }

    public double getPosition() {
        return position;
    }

    public double getEncoderPosition() {
        return lower.getCurrentPosition();
    }

    public void setPosition(double position) {
        this.position = position;
    }

    public boolean isLiftDown() {
        //return !limitSwitch.getState();
        return Math.abs(getEncoderPosition()) < 30;
    }

    public void setPower(double power) {
        lower.setPower(power);
        middle.setPower(power);
        upper.setPower(power);
    }

    double manPower = 0.0;
    public void setManuealPower(double power) {
        manPower = power;
    }

    @Override
    public void init() throws InterruptedException {
        upper.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lower.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        middle.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        upper.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        lower.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        middle.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        upper.setDirection(DcMotorSimple.Direction.REVERSE);
        lower.setDirection(DcMotorSimple.Direction.REVERSE);
        middle.setDirection(DcMotorSimple.Direction.REVERSE);

        //upper.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //lower.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        //middle.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //limitSwitch.setMode(DigitalChannel.Mode.INPUT);
    }

    @Override
    public void update() throws InterruptedException {
        if(!manueal) {
            runToPosition(position);
        }
        else {
            setPower(manPower);
        }
    }


}
