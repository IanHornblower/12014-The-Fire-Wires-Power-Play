package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.outoftheboxrobotics.photoncore.Neutrino.Rev2MSensor.Rev2mDistanceSensorEx;
import com.qualcomm.robotcore.hardware.ColorRangeSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.interfaces.Subsystem;

public class LineAlignment implements Subsystem {

    public ColorRangeSensor left, right;
    public Rev2mDistanceSensorEx distance;

    public Robot robot;
    public HardwareMap hardwareMap;

    public static double wallTolerance = 195;

    public LineAlignment(Robot robot) {
     this.robot = robot;
     this.hardwareMap = robot.hwMap;

     left = hardwareMap.get(ColorRangeSensor.class, "leftColor");
     right = hardwareMap.get(ColorRangeSensor.class, "rightColor");

     distance = hardwareMap.get(Rev2mDistanceSensorEx.class, "distanceSensor");
    }

    public double getDistance() {
        return distance.getDistance(DistanceUnit.MM);
    }

    public double getLeftValue() {
        return  left.blue();
    }

    public double getRightValue() {
        return  right.blue();
    }

    public boolean getRight() {
        return getRightValue() > 110;
    }

    public boolean getLeft() {
        return getLeftValue() > 110;
    }

    @Override
    public void init() throws InterruptedException {

    }

    @Override
    public void update() throws InterruptedException {

    }
}
