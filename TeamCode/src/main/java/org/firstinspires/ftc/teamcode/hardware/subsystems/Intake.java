package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.outoftheboxrobotics.photoncore.Neutrino.RevColorSensor.RevColorSensorV3Ex;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.CRServo;
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

    CRServo bl, br, fr, fl;
    public static double multiplier = 1;
    public static double speed = 0.6;
    public static double coneThreshold = 40;

    public RevColorSensorV3Ex front, back;



    public Intake(Robot robot) {
        HardwareMap hwMap = robot.hwMap;

       bl = hwMap.get(CRServo.class, "bl_intake");
       br = hwMap.get(CRServo.class, "br_intake");
       fr = hwMap.get(CRServo.class, "fr_intake");
       fl = hwMap.get(CRServo.class, "fl_intake");

       front = hwMap.get(RevColorSensorV3Ex.class, "front");
       back = hwMap.get(RevColorSensorV3Ex.class, "back");
    }

    public enum DIRECTION {
        front,
        back
    }

    public  DIRECTION direction = DIRECTION.front;

    public void setDirection(DIRECTION d) {
        direction = d;
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
        bl.setPower(power);
        br.setPower(-power);
        fr.setPower(power);
        fl.setPower(-power);
    }

    public boolean hasCone() {
        if(direction == DIRECTION.front) {
            return back.getDistance(DistanceUnit.MM) < 35;
        }else {
            return front.getDistance(DistanceUnit.MM) < 35;
        }
    }

    @Override
    public void init() throws InterruptedException {

    }

    @Override
    public void update() throws InterruptedException {

    }
}
