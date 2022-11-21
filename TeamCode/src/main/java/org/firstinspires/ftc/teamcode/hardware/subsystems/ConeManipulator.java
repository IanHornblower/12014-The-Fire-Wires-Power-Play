package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.interfaces.Subsystem;

@Config
public class ConeManipulator implements Subsystem {

    public static double leftBound = 0;
    public static double rightBound = 1;

    Servo left;
    Servo right;
    CRServo intake;

    public ConeManipulator(Robot robot) {
        HardwareMap hwMap = robot.hwMap;

        left = hwMap.get(Servo.class, "leftServo");
        right = hwMap.get(Servo.class, "rightServo");

        intake = hwMap.get(CRServo.class, "intakeServo");
    }


    public void setLeft(double position) {
        left.setPosition(position);
    }

    public void setRight(double position) {
        right.setPosition(position);
    }

    public void setServoPositions(double position) {
        double leftPosition = position - 1;

        if (position - 1 < 0) {
          leftPosition = position * -1;
        }

        left.setPosition(leftPosition);
        right.setPosition(position);
    }


    @Override
    public void init() throws InterruptedException {

    }

    @Override
    public void update() throws InterruptedException {

    }
}
