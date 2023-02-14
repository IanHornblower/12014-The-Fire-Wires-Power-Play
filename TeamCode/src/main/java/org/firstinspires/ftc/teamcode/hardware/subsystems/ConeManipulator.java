package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.teamcode.ActionSystem.TeleOpAction;
import org.firstinspires.ftc.teamcode.ActionSystem.actions.CustomAction;

import org.firstinspires.ftc.teamcode.Control.SqrtCoefficients;
import org.firstinspires.ftc.teamcode.Control.SqrtControl;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.interfaces.Subsystem;

import java.util.function.DoubleSupplier;

@Config
public class ConeManipulator implements Subsystem {
    public DcMotorEx fourbar;
    Servo intake;

    public static double kP = 0.0, kD = 0, h = 0;
    public static double kF = 0.2;

    public static double speed = 0.0;

    public static double grabberOpen = 1;
    public static double grabberTransfer = 0.5;
    public static double grabberCorrect = 0.3;
    public static double grabberClose = 0;

    double position = 0;

    //SqrtControl fourbar = new SqrtControl(new SqrtCoefficients(kP, kD, h));
    //BasicPID loop = new BasicPID(new PIDCoefficients(0.0, 0.0, 0));
   // SqrtControl mainLoop = new SqrtControl(new SqrtCoefficients(kP, kD, h));
    double max = 0.6;

    DoubleSupplier power = ()-> 0.0;

    public TeleOpAction[] actions;
    public TeleOpAction grabCone;
    public TeleOpAction dropConeAndReturn;
    public TeleOpAction raiseToTop;
    public TeleOpAction raiseToMid;
    public TeleOpAction raiseToLow;
    public TeleOpAction raiseToGround;
    public TeleOpAction retractOdom;
    public TeleOpAction releaseOdom;
    public TeleOpAction correctCone;
    public TeleOpAction raiseLift;
    public TeleOpAction dropConeAndReturnFar;
    public TeleOpAction grabConeFromStack;

    Robot robot;

    public double error;

    public ConeManipulator(Robot robot) {
        HardwareMap hwMap = robot.hwMap;
        this.robot = robot;

        fourbar = hwMap.get(DcMotorEx.class, "4bar");

        fourbar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        intake = hwMap.get(Servo.class, "intakeServo");

       // Actions
       grabCone = new TeleOpAction(robot);
       dropConeAndReturn = new TeleOpAction(robot);
       raiseToTop = new TeleOpAction(robot);
       raiseToMid = new TeleOpAction(robot);
       raiseToLow = new TeleOpAction(robot);
       raiseToGround = new TeleOpAction(robot);
       retractOdom = new TeleOpAction(robot);
       releaseOdom = new TeleOpAction(robot);
       correctCone = new TeleOpAction(robot);
       raiseLift = new TeleOpAction(robot);
       dropConeAndReturnFar = new TeleOpAction(robot);
       grabConeFromStack = new TeleOpAction(robot);

    }

    public enum V4BPreset {
        VERTICAL(1350, 1350), // //
        PRIME(260, 2500),  // //
        GRAB(85, 2750), //`//
        DROP(1720, 1100), // //
        FAR_DROP(2200, 550), // //
        EMERGENCY_SMALL(1300, 0), // /X
        TeleOpDROP(1480, 1100), // /x
        CORRECT_RIGHT(1900, 850), // //
        STACK1(510, 510),
        STACK2(420, 420),
        STACK3(350, 350),
        STACK4(250, 250);
        //CORRECT_CONE();

        double front, back;

        V4BPreset(double front, double back) {
            this.front = front;
            this.back = back;
        }

        public double getFront() {
            return front;
        }

        public double getBack() {
            return back;
        }
     }

     public void definePower(DoubleSupplier power) {
        this.power = power;
     }

    public void setPosition(double position) {
        this.position = position;
    }

    public void resetEnc() {
        fourbar.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fourbar.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void runToPosition(double position) {
        error = position - fourbar.getCurrentPosition();

        if(Math.abs(error) < 50) {
            fourbar.setPower(0.0);
        }
        else if(Math.abs(error) < 200) {
            fourbar.setPower(Math.signum(error) * 0.5);
        }
        else {
            fourbar.setPower(Math.signum(error) * 0.8);
        }
    }

    public void runToPosition(double position, boolean holdPosition) { // Maybe use to mitigate 4bar dropping issues
        if(fourbar.getZeroPowerBehavior() == DcMotor.ZeroPowerBehavior.BRAKE) {
            fourbar.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        }

        double error = position - fourbar.getCurrentPosition();
        double angle = calc(fourbar.getCurrentPosition());
        double Kcos = Math.cos(angle);

        if(Math.abs(error) < 50) {
            fourbar.setPower(0.0 + Kcos);
        }
        else if(Math.abs(error) < 200) {
            fourbar.setPower(Math.signum(error) * 0.5 + Kcos);
        }
        else {
            fourbar.setPower(Math.signum(error) * 0.8 + Kcos);
        }
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
        if(robot.tempSideFlip.getAsBoolean() && positions == V4BPreset.DROP) {
                if(robot.intake.direction == Intake.DIRECTION.front) {
                    setPosition(positions.getBack());
                }
                else {
                    setPosition(positions.getFront());
                }
        }
        else {
            if(robot.intake.direction == Intake.DIRECTION.front) {
                setPosition(positions.getFront());
            }
            else {
                setPosition(positions.getBack());
            }
        }


    }

    private void raiseLiftInit() {
        raiseLift.addCustomAction(()-> robot.lift.setPosition(robot.lift.getEncoderPosition() + 220));
        raiseLift.addWait(0.2);
        raiseLift.addCustomAction(()-> setPosition(V4BPreset.FAR_DROP));
    }

    private void correctConeInit() {
        correctCone.addCustomAction(()-> setPosition(V4BPreset.GRAB));
        correctCone.addWait(0.3);
        correctCone.addCustomAction(()-> intake.setPosition(grabberCorrect));
    }

    private void grabConeInit() {
        grabCone.addCustomAction(()-> setPosition(V4BPreset.GRAB));
        //grabCone.addWait(0.2);
        grabCone.addCustomAction(()-> close());
        grabCone.addWait(0.4); // 0.28
        grabCone.addCustomAction(()-> setPosition(V4BPreset.VERTICAL));
    }

    private void dropConeAndReturnInit() {
        dropConeAndReturn.addCustomAction(()-> setPosition(V4BPreset.CORRECT_RIGHT));
        dropConeAndReturn.addWait(0.1);
        dropConeAndReturn.addCustomAction(()-> open());
        dropConeAndReturn.addWait(0.15); // causes pull up
        dropConeAndReturn.addCustomAction(()-> setPosition(V4BPreset.PRIME));
        dropConeAndReturn.addCustomAction(()-> close());
        dropConeAndReturn.addWait(0.2); // was 0.4 // delay for returning slides
        dropConeAndReturn.addAction(new CustomAction(()-> robot.lift.setPosition(0)));
        dropConeAndReturn.addWait(0);
        dropConeAndReturn.addAction(new CustomAction(()-> open()));
    }

    private void dropConeAndReturnFarInit() {
        dropConeAndReturnFar.addCustomAction(()-> open());
        dropConeAndReturnFar.addWait(0.15); // causes pull up
        dropConeAndReturnFar.addCustomAction(()-> setPosition(V4BPreset.PRIME));
        dropConeAndReturnFar.addCustomAction(()-> close());
        dropConeAndReturnFar.addWait(0.4);
        dropConeAndReturnFar.addAction(new CustomAction(()-> robot.lift.setPosition(0)));
        dropConeAndReturnFar.addWait(0);
        dropConeAndReturnFar.addAction(new CustomAction(()-> open()));
    }

    private void raiseToTopInit() {
        raiseToTop.addAction(new CustomAction(()-> close()));
        raiseToTop.addCustomAction(()-> setPosition(V4BPreset.DROP));
        raiseToTop.addAction(new CustomAction(()-> robot.lift.setPosition(Lift.highPole)));
    }

    private void raiseToMidInit() {
        raiseToMid.addAction(new CustomAction(()-> close()));
        raiseToMid.addCustomAction(()-> setPosition(V4BPreset.DROP));
        raiseToMid.addAction(new CustomAction(()-> robot.lift.setPosition(Lift.middlePole)));
    }

    private void raiseToLowInit() {
        raiseToLow.addAction(new CustomAction(()-> close()));
        raiseToLow.addCustomAction(()-> setPosition(V4BPreset.DROP));
        raiseToLow.addAction(new CustomAction(()-> robot.lift.setPosition(Lift.smallPole)));
    }

    private void raiseToGround() {
        raiseToGround.addAction(new CustomAction(()-> {
            close();
            if(robot.intake.direction == Intake.DIRECTION.back) {
                robot.intake.setDirection(Intake.DIRECTION.front);
            }
            else {
                robot.intake.setDirection(Intake.DIRECTION.back);
            }

        }));
        raiseToGround.addCustomAction(()-> {
            setPosition(V4BPreset.GRAB);

            if(robot.intake.direction == Intake.DIRECTION.back) {
                robot.intake.setDirection(Intake.DIRECTION.front);
            }
            else {
                robot.intake.setDirection(Intake.DIRECTION.back);
            }
        });
        //raiseToGround.addWait(0.2); // Add Back in if it twiches or sum
        //raiseToGround.addAction(new CustomAction(()-> {
        //    if(robot.intake.direction == Intake.DIRECTION.back) {
        //        robot.intake.setDirection(Intake.DIRECTION.front);
        //    }
        //    else {
        //        robot.intake.setDirection(Intake.DIRECTION.back);
        //    }
        //}));
    }

    private void grabConeFromStackInit() {
        grabConeFromStack.addCustomAction(()-> open());
        grabConeFromStack.addCustomAction(()-> {
            switch (robot.coneStack) {
                case 5:
                    robot.lift.setPosition(Lift.cone5);
                    break;
                case 4:
                    robot.lift.setPosition(Lift.cone4);
                    break;
                case 3:
                    robot.lift.setPosition(Lift.cone3);
                    break;
                case 2:
                    robot.lift.setPosition(Lift.cone2);
                    break;
            }
        });
        //grabConeFromStack.addCustomAction(()-> setPosition(V4BPreset.PRIME));
    }

    private void retractOdomInit() {
        retractOdom.addCustomAction(()-> robot.retract());
    }

    private void releaseOdomInit() {
        retractOdom.addCustomAction(()-> robot.release());
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
        correctConeInit();
        raiseLiftInit();
        dropConeAndReturnFarInit();
        grabConeFromStackInit();
    }

    @Override
    public void init() throws InterruptedException {
        resetEnc();
        initActions();

        actions = new TeleOpAction[] {grabConeFromStack, dropConeAndReturnFar, raiseLift, correctCone, grabCone, dropConeAndReturn, raiseToTop, raiseToMid, raiseToLow, raiseToGround, retractOdom, releaseOdom};
    }

    public boolean auto = true;

    public void autoUpdate() throws InterruptedException {
        for(TeleOpAction action : actions) {
            action.run();
        }

        runToPosition(position);
    }

    @Override
    public void update() throws InterruptedException {
        if(auto) {
            autoUpdate();
        }
        else {
          fourbar.setPower(power.getAsDouble());
        }
    }

    public double getAngle() {
        return calc(fourbar.getCurrentPosition());
    }

    public static double calc(double enc) {
        double slope = (Math.toRadians(180) - Math.toRadians(55))/ -1350;

        return slope * (enc - 1350) + Math.toRadians(180);
    }
}
