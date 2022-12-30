package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.outoftheboxrobotics.photoncore.Neutrino.RevColorSensor.RevColorSensorV3Ex;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ActionSystem.ActionSequenceRunner;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.interfaces.Subsystem;

import kotlin.text.CharDirectionality;

@Config
public class Intake implements Subsystem {

    DcMotor left, right;
    public RevColorSensorV3Ex detector;
    double left_p, right_p;
    public static double multiplier = 1;
    public static double speed = 0.6;
    public static double coneThreshold = 40;
    public ActionSequenceRunner actionRunner;

    public Intake(Robot robot) {
        HardwareMap hwMap = robot.hwMap;

        left = hwMap.get(DcMotor.class, "leftIntake");
        right = hwMap.get(DcMotor.class, "rightIntake");

        detector = hwMap.get(RevColorSensorV3Ex.class, "detector");
    }

    public void start() {
        setPower(speed);
    }

    public void reverse() {
        setPower(-speed);
    }

    public void stop() {
        setPower(0.0);
    }

    public void setPower(double power) {
        left_p = power * -multiplier;
        right_p = power * multiplier;
    }

    public boolean hasCone() {
        return detector.getDistance(DistanceUnit.MM) < coneThreshold;
    }

    @Override
    public void init() throws InterruptedException {

    }

    @Override
    public void update() throws InterruptedException {
        left.setPower(left_p);
        right.setPower(right_p);
    }
}
