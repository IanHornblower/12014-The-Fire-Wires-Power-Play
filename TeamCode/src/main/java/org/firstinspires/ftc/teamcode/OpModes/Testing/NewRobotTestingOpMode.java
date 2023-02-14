
package org.firstinspires.ftc.teamcode.OpModes.Testing;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.profile.MotionProfile;
import com.acmerobotics.roadrunner.profile.MotionProfileBuilder;
import com.acmerobotics.roadrunner.profile.MotionProfileGenerator;
import com.acmerobotics.roadrunner.profile.MotionState;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.ActionSystem.Action;
import org.firstinspires.ftc.teamcode.ActionSystem.ActionSequence;
import org.firstinspires.ftc.teamcode.ActionSystem.ActionSequenceRunner;
import org.firstinspires.ftc.teamcode.ActionSystem.actions.V4BSetPosition;
import org.firstinspires.ftc.teamcode.Control.SqrtCoefficients;
import org.firstinspires.ftc.teamcode.Control.SqrtControl;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.ConeManipulator;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Intake;
import org.firstinspires.ftc.teamcode.util.Timer;

@Config
@TeleOp(name = "New Robot Testing")
public class NewRobotTestingOpMode extends LinearOpMode {


    // Intake
    // Lift
    // Minip

    public static double kP = 0.0, kD = 0.0, max = 0.5, h = 0, kI = 0.0;

    MotionProfile fourbarProfile;

    BasicPID loop = new BasicPID(new PIDCoefficients(kP, kI, kD));

    public static double liftSpeed = 1;

    public static double angle = 0;
    public static double liftPos = 0;
    public static double ticks180 = 275;

    public static double kF = 0.2;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        telemetry.setItemSeparator(" >> ");

        Robot rob = new Robot(hardwareMap, telemetry, Robot.OPMODE_TYPE.AUTO);

        rob.init();
        telemetry = rob.getTelemetry();

        waitForStart();

        rob.intake.setDirection(Intake.DIRECTION.back);

        double prevAngle = angle;
        fourbarProfile = MotionProfileGenerator.generateSimpleMotionProfile(new MotionState(Math.toRadians(55), 0, 0), new MotionState(Math.toRadians(angle), 0, 0), Math.toRadians(30), Math.toRadians(30));

        Timer timer = new Timer();

        //rob.lift.setModeAutomatic();
        rob.lift.setModeManuel();
        rob.coneManipulator.auto = true;
        rob.coneManipulator.definePower(()->gamepad1.left_stick_y * liftSpeed);

        ActionSequence as = new ActionSequence();
        ActionSequenceRunner asr = new ActionSequenceRunner(rob);

       // as.addAction(new V4BSetPosition(rob, 1000));

        asr.setActionSequence(as);

        timer.start();
        while(opModeIsActive() && !isStopRequested()) {

            /*
            if(gamepad1.left_bumper) {
                rob.intake.setPower(-1);
            }
            else if(gamepad1.right_bumper) {
                rob.intake.setPower(1);
            }
            else {
                rob.intake.setPower(0);
            }

             */

            rob.lift.regeneratePID();

            rob.lift.setPosition(liftPos);
            rob.coneManipulator.setPosition(liftPos);

            double liftPower = (liftSpeed * gamepad1.right_trigger) + (-liftSpeed * gamepad1.left_trigger);
            rob.lift.setManuealPower(liftPower);


            if(gamepad1.left_bumper) {
                rob.release();
            }
            if(gamepad1.right_bumper) {
                rob.retract();
            }



            //rob.driveTrain.setWeightedDrivePower(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x*0.8);


            //rob.localizer.leftRetract.setPower(intakeSpeed);
            //rob.localizer.rightRetract.setPower(intakeSpeed);



            //rob.lift.setPower(liftPower);


            if(gamepad1.cross) {
                rob.coneManipulator.close();
            }
            if(gamepad1.circle) {
                rob.coneManipulator.open();
            }



           // rob.coneManipulator.setPosition(position);

            //rob.lift.lower.setPower(lowerPower);
            //rob.lift.upper.setPower(upperPower);



            //double getTickFromAngle = (angle * ticks180) / Math.toRadians(180);
            //double calcFF = kF * Math.sin(Math.toRadians(angle));
            //rob.coneManipulator.fourbar.setPower(calcFF + loop.calculate(yuh.getX(), getAngleFromTicks));


            //rob.coneManipulator.setPosition(angle, gamepad1.left_stick_y * liftSpeed);

            //rob.coneManipulator.setPosition(angle);
            //telemetry.addData("refrace", angle);
            //telemetry.addData("enc", rob.coneManipulator.fourbar.getCurrentPosition());

            telemetry.addData("pos", liftPos);
            telemetry.addData("arm enc", rob.coneManipulator.fourbar.getCurrentPosition());
            telemetry.addData("lift enc", rob.lift.getEncoderPosition());
            //telemetry.addData("Amps",rob.lift.lower.getCurrent(CurrentUnit.MILLIAMPS));
            //telemetry.addData("lift enc", rob.lift.getEncoderPosition());
            //telemetry.addData("lift power", liftPower);
            telemetry.update();

            try {
                rob.update();
            } catch (Exception e) {
                e.printStackTrace();
            }
        }




    }

    public static double calc(double enc) {
        double slope = (Math.toRadians(180) - Math.toRadians(55))/ 275;

        return slope * (enc - 275) + Math.toRadians(180);
    }
}
