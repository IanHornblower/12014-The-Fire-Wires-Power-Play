package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.checkerframework.checker.units.qual.A;
import org.checkerframework.checker.units.qual.C;
import org.firstinspires.ftc.teamcode.ActionSystem.ActionSequence;
import org.firstinspires.ftc.teamcode.ActionSystem.TeleOpAction;
import org.firstinspires.ftc.teamcode.ActionSystem.actions.ConeTurn;
import org.firstinspires.ftc.teamcode.ActionSystem.actions.CustomAction;
import org.firstinspires.ftc.teamcode.ActionSystem.actions.LiftSetPosition;
import org.firstinspires.ftc.teamcode.ActionSystem.actions.Wait;
import org.firstinspires.ftc.teamcode.ActionSystem.actions.WaitFor;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.interfaces.Subsystem;

@Config
public class ConeManipulator implements Subsystem {
    Servo left;
    Servo right;
    Servo intake;

    ServoImplEx leftS;
    ServoImplEx rightS;


    public static double grabberOpen = 0.7;
    public static double grabberTransfer = 0.42;
    public static double grabberClose = 0.42;

    public TeleOpAction[] actions;
    public TeleOpAction grabCone;
    public TeleOpAction dropConeAndReturn;
    public TeleOpAction raiseToTop;
    public TeleOpAction raiseToMid;
    public TeleOpAction raiseToLow;
    public TeleOpAction raiseToGround;
    public TeleOpAction retractOdom;
    public TeleOpAction releaseOdom;

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
       raiseToMid = new TeleOpAction(robot);
       raiseToLow = new TeleOpAction(robot);
       raiseToGround = new TeleOpAction(robot);
       retractOdom = new TeleOpAction(robot);
       releaseOdom = new TeleOpAction(robot);
    }

    public enum V4BPreset {
        VERTICAL(0.5, 0.5),
        PRIME(0.9, 0.1),
        DROP(0.4, 0.6),
        CORRECT_RIGHT(0, 0),
        CORRECT_LEFT(0, 0);

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

    public void middle() {
        intake.setPosition(grabberTransfer);
    }

    public void setPosition(V4BPreset positions) {
        double[] position = new double[] {positions.left, positions.right};

        if(robot.tempSideFlip.getAsBoolean()) {
            if (robot.intake.direction == Intake.DIRECTION.back) {
                switch (positions) {
                    case VERTICAL:
                        position = new double[] {0.53, 0.47};
                        break;
                    case PRIME:
                        position = new double[] {0.95, 0.05};
                        break;
                    case DROP:
                        position = new double[] {0.4, 0.6};
                        break;
                    case CORRECT_RIGHT:
                        position = new double[] {0.35, 0.7};
                        break;
                    case CORRECT_LEFT:
                        break;
                }
            }
            else {
                switch (positions) {
                    case VERTICAL:
                        position = new double[] {0.53, 0.47};
                        break;
                    case PRIME:
                        position = new double[] {0.08, 0.92};
                        break;
                    case DROP:
                        position = new double[] {0.68, 0.4};
                        break;
                    case CORRECT_RIGHT:
                        position = new double[] {0.75, 0.35};
                        break;
                    case CORRECT_LEFT:
                        break;
                }
            }
        }
        else {
            if (robot.intake.direction == Intake.DIRECTION.front) {
                switch (positions) {
                    case VERTICAL:
                        position = new double[] {0.53, 0.47};
                        break;
                    case PRIME:
                        position = new double[] {0.95, 0.05};
                        break;
                    case DROP:
                        position = new double[] {0.4, 0.6};
                        break;
                    case CORRECT_RIGHT:
                        position = new double[] {0.35, 0.7};
                        break;
                    case CORRECT_LEFT:
                        break;
                }
            }
            else {
                switch (positions) {
                    case VERTICAL:
                        position = new double[] {0.53, 0.47};
                        break;
                    case PRIME:
                        position = new double[] {0.08, 0.92};
                        break;
                    case DROP:
                        position = new double[] {0.68, 0.4};
                        break;
                    case CORRECT_RIGHT:
                        position = new double[] {0.75, 0.35};
                    case CORRECT_LEFT:
                        break;
                }
            }
        }
        setServoPositions(position);

    }

    public void setServoPositions(double[] positions) {
        setLeft(positions[0]);
        setRight(positions[1]);
    }

    private void grabConeInit() {
        grabCone.addCustomAction(()-> close());
        grabCone.addWait(0.28);
        grabCone.addAction(new CustomAction(()-> setPosition(V4BPreset.VERTICAL)));
    }

    private void dropConeAndReturnInit() {
        dropConeAndReturn.addCustomAction(()-> setPosition(V4BPreset.CORRECT_RIGHT));
        dropConeAndReturn.addWait(0.1);
        dropConeAndReturn.addCustomAction(()-> open());
        dropConeAndReturn.addWait(0.05);
        dropConeAndReturn.addAction(new CustomAction(()-> setPosition(V4BPreset.PRIME)));
        dropConeAndReturn.addCustomAction(()-> close());
        dropConeAndReturn.addWait(0.4);
        dropConeAndReturn.addAction(new CustomAction(()-> robot.lift.setPosition(0)));
        dropConeAndReturn.addWait(0);
        dropConeAndReturn.addAction(new CustomAction(()-> open()));
    }

    private void raiseToTopInit() {
        raiseToTop.addAction(new CustomAction(()-> close()));
        raiseToTop.addAction(new CustomAction(()-> setPosition(V4BPreset.DROP)));
        raiseToTop.addAction(new CustomAction(()-> robot.lift.setPosition(Lift.highPole)));
    }

    private void raiseToMidInit() {
        raiseToMid.addAction(new CustomAction(()-> close()));
        raiseToMid.addAction(new CustomAction(()-> setPosition(V4BPreset.DROP)));
        raiseToMid.addAction(new CustomAction(()-> robot.lift.setPosition(Lift.middlePole)));
    }

    private void raiseToLowInit() {
        raiseToLow.addAction(new CustomAction(()-> close()));
        raiseToLow.addAction(new CustomAction(()-> setPosition(V4BPreset.DROP)));
        raiseToLow.addAction(new CustomAction(()-> robot.lift.setPosition(Lift.smallPole)));
    }

    private void raiseToGround() {
        raiseToGround.addCustomAction(()-> setPosition(V4BPreset.PRIME));
        raiseToGround.addCustomAction(()-> open());
    }

    private void retractOdomInit() {
        retractOdom.addAction(new CustomAction(() -> {
            robot.localizer.leftRetract.setPower(-0.5);
            robot.localizer.rightRetract.setPower(-0.5);
            robot.localizer.latteralRetract.setPower(-0.5);
        }));
        retractOdom.addWait(0.7);
        retractOdom.addAction(new CustomAction(() -> {
            robot.localizer.leftRetract.setPower(0.0);
            robot.localizer.rightRetract.setPower(0.0);
            robot.localizer.latteralRetract.setPower(0.0);
        }));
    }

    private void releaseOdomInit() {
        releaseOdom.addAction(new CustomAction(() -> {
            robot.localizer.leftRetract.setPower(0.5);
            robot.localizer.rightRetract.setPower(0.5);
            robot.localizer.latteralRetract.setPower(0.5);
        }));
        releaseOdom.addWait(0.7);
        releaseOdom.addAction(new CustomAction(() -> {
            robot.localizer.leftRetract.setPower(0.0);
            robot.localizer.rightRetract.setPower(0.0);
            robot.localizer.latteralRetract.setPower(0.0);
        }));
    }

    private void initActions() {
        grabConeInit();
        dropConeAndReturnInit();
        raiseToLowInit();
        raiseToMidInit();
        raiseToTopInit();
        raiseToGround();
        retractOdomInit();
        releaseOdomInit();
    }

    @Override
    public void init() throws InterruptedException {
        initActions();

        actions = new TeleOpAction[] {grabCone, dropConeAndReturn, raiseToTop, raiseToMid, raiseToLow, raiseToGround, retractOdom, releaseOdom};
    }

    @Override
    public void update() throws InterruptedException {
        for(TeleOpAction action : actions) {
            action.run();
        }
    }
}
