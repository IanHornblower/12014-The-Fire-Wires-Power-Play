package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.ActionSystem.ActionSequenceRunner;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.interfaces.Subsystem;

public class Jimmy implements Subsystem {

    public ServoImplEx joe;
    public CRServo Jameson;
    public ActionSequenceRunner KoolAidAF1;
    double power = 0.0;

    public Jimmy(Robot robot) {
        Jameson = robot.hwMap.get(CRServo.class, "jameson");
    }

    public void setPower(double power) {
        this.power = power;
    }

    public void reverse() {
        if(Jameson.getDirection() == DcMotorSimple.Direction.REVERSE) {
            Jameson.setDirection(DcMotorSimple.Direction.FORWARD);
        }
        else {
            Jameson.setDirection(DcMotorSimple.Direction.REVERSE);
        }

        ///joe.setPwmRange(new PwmControl.PwmRange(500, 2500));
    }

    @Override
    public void init() throws InterruptedException {
        Jameson.setDirection(DcMotorSimple.Direction.FORWARD);
    }

    @Override
    public void update() throws InterruptedException {
        Jameson.setPower(power);
    }
}
