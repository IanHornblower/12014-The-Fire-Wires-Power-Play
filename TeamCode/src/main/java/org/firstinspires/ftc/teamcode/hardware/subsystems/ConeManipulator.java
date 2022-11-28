package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.ActionSystem.ActionSequence;
import org.firstinspires.ftc.teamcode.ActionSystem.TeleOpAction;
import org.firstinspires.ftc.teamcode.ActionSystem.actions.CustomAction;
import org.firstinspires.ftc.teamcode.ActionSystem.actions.LiftSetPosition;
import org.firstinspires.ftc.teamcode.ActionSystem.actions.Wait;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.interfaces.Subsystem;

@Config
public class ConeManipulator implements Subsystem {
    Servo left;
    Servo right;
    Servo intake;

    ServoImplEx leftS;
    ServoImplEx rightS;


    public static double grabberOpen = 0.5;
    public static double grabberClose = 0.3 ;

    TeleOpAction[] actions;
    public TeleOpAction grabCone;
    public TeleOpAction dropConeAndReturn;
    public TeleOpAction raiseToTop;
    public TeleOpAction raiseToMid;
    public TeleOpAction raiseToLow;

    Robot robot;

    public ConeManipulator(Robot robot) {
        HardwareMap hwMap = robot.hwMap;
        this.robot = robot;

        left = hwMap.get(Servo.class, "leftServo");
        right = hwMap.get(Servo.class, "rightServo");

        intake = hwMap.get(Servo.class, "intakeServo");

       leftS = hwMap.get(ServoImplEx.class, "leftServo");
       rightS  = hwMap.get(ServoImplEx.class, "rightServo");

       // Actions
       grabCone = new TeleOpAction(robot);
       dropConeAndReturn = new TeleOpAction(robot);
       raiseToTop = new TeleOpAction(robot);
       raiseToLow = new TeleOpAction(robot);
       raiseToLow = new TeleOpAction(robot);
    }

    public enum V4BPreset {
        IN_MOST(0.85, 0.04),
        INNER_PRIME(0.75, 0.15),
        Vertiacal(0.44, 0.45),
        //DROP(0.3, 0.55),
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

    private void grabConeInit() {
        grabCone.addAction(new CustomAction(()-> {
            setPosition(ConeManipulator.V4BPreset.INNER_PRIME);
            open();
        }));
        grabCone.addWait(0.2);
        grabCone.addAction(new CustomAction(()-> setPosition(ConeManipulator.V4BPreset.IN_MOST)));
        grabCone.addWait(0.3);
        grabCone.addAction(new CustomAction(this::close));
        grabCone.addWait(0.2);
        grabCone.addAction(new CustomAction(()-> setPosition(ConeManipulator.V4BPreset.Vertiacal)));
    }

    private void dropConeAndReturnInit() {
        dropConeAndReturn.addAction(new CustomAction(()-> robot.coneManipulator.open()));
        dropConeAndReturn.addWait(0.2);
        dropConeAndReturn.addAction(new CustomAction(()-> setPosition(V4BPreset.INNER_PRIME)));
        dropConeAndReturn.addAction(new LiftSetPosition(robot, Lift.lowResting));
        //dropConeAndReturn.addAction(new CustomAction(()-> robot.lift.setPower(-0.1)));
        //dropConeAndReturn.addWait(0.3);
        //dropConeAndReturn.addAction(new CustomAction(()-> robot.lift.setPower(0)));
    }

    private void raiseToTopInit() {
        raiseToTop.addAction(new LiftSetPosition(robot, Lift.highPole));
        raiseToTop.addCustomAction(()->robot.coneManipulator.setPosition(V4BPreset.DROP));

    }

    private void raiseToMidInit() {
        raiseToMid.addAction(new LiftSetPosition(robot, Lift.middlePole));
        raiseToMid.addCustomAction(()->robot.coneManipulator.setPosition(V4BPreset.DROP));
    }

    private void raiseToLowInit() {
        raiseToLow.addAction(new LiftSetPosition(robot, Lift.smallPole));
        raiseToLow.addCustomAction(()->robot.coneManipulator.setPosition(V4BPreset.DROP));
    }

    private void initActions() {
        grabConeInit();
        dropConeAndReturnInit();
    }

    @Override
    public void init() throws InterruptedException {
        initActions();

        actions = new TeleOpAction[] {grabCone, dropConeAndReturn, raiseToTop, raiseToMid, raiseToLow};
    }

    @Override
    public void update() throws Exception {
        for(TeleOpAction action : actions) {
            action.run();
        }
    }
}
