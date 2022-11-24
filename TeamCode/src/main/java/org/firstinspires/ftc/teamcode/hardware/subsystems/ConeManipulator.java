package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.interfaces.Subsystem;

@Config
public class ConeManipulator implements Subsystem {
    Servo left;
    Servo right;
    Servo intake;

    ServoImplEx leftS;
    ServoImplEx rightS;


    public static double grabberOpen = 1;
    public static double grabberClose = 0.8 ;

    public ConeManipulator(Robot robot) {
        HardwareMap hwMap = robot.hwMap;

        left = hwMap.get(Servo.class, "leftServo");
        right = hwMap.get(Servo.class, "rightServo");

        intake = hwMap.get(Servo.class, "intakeServo");

       leftS = hwMap.get(ServoImplEx.class, "leftServo");
       rightS  = hwMap.get(ServoImplEx.class, "rightServo");
    }

    public enum V4BPreset {
        IN_MOST(0.85, 0.04),
        INNER_PRIME(0.75, 0.15),
        Vertiacal(0.44, 0.45),
        DROP(0.2, 0.68),
        GROUND_LEVEL(0.02, 0.86);

        double left, right;

        V4BPreset(double left, double right) {
            this.left = left;
            this.right = right;
        }

        public double getLeft() {
            return left;
        }

        public double getRight() {
            return right;
        }

        public double[] getServoPositions() {
            return new double[] {left, right};
        }
     }

    public void setLeft(double position) {
        left.setPosition(position);
    }

    public void setRight(double position) {
        right.setPosition(position);
    }

    public void close() {
        intake.setPosition(grabberClose);
    }

    public void open() {
        intake.setPosition(grabberOpen);
    }

    public void setPosition(V4BPreset positions) {
        setServoPositions(positions.getServoPositions());
    }

    public void setServoPositions(double[] positions) {
        setLeft(positions[0]);
        setRight(positions[1]);
    }


    @Override
    public void init() throws InterruptedException {
        leftS.setPwmRange(new PwmControl.PwmRange(500, 2500));
        rightS.setPwmRange(new PwmControl.PwmRange(500, 2500));
    }

    @Override
    public void update() throws InterruptedException {

    }
}
