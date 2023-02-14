
package org.firstinspires.ftc.teamcode.OpModes.Actual;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.ConeManipulator;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Intake;
import org.firstinspires.ftc.teamcode.util.Color;
import org.firstinspires.ftc.teamcode.util.Timer;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "TeleOp")
public class StateTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        telemetry.setItemSeparator(" >> ");

        Robot rob = new Robot(hardwareMap, telemetry, Robot.OPMODE_TYPE.TELEOP);

        telemetry = rob.getTelemetry();

        rob.init();

        Timer timer = new Timer();
        timer.reset();

        rob.coneManipulator.definePower(()-> gamepad2.right_stick_y);

        rob.coneManipulator.auto = false;

        waitForStart();

        // My Gamepad Map
        /*
        Left Trigger -- Simple Drop
        Right Trigger -- AUTO Grab

        Left Bumper  - Drop Release
        Right Bumber - Grab

        Cross
        Triangle -- nun
        Square -- Switch To Manual Lift
        Circle -- Switch To Auto Lift
        Cross -- Reset Lift Enc


        DPAD
            - Left -- Low
            - Right -- High
            - Up -- Mid
            - Down --

        Joystick Left
        Joystick Right
        Joystick Left Btn
        Joystick Right Btn

         */

        rob.coneManipulator.raiseToTop.start(()-> gamepad2.dpad_right); // GROUND
        rob.coneManipulator.raiseToMid.start(()-> gamepad2.dpad_up); // GROUND
        rob.coneManipulator.raiseToLow.start(()-> gamepad2.dpad_left); // GROUND
        rob.coneManipulator.dropConeAndReturn.start(()-> gamepad2.left_bumper); // GROUND
        rob.coneManipulator.grabCone.start(()-> gamepad2.right_bumper);
        rob.coneManipulator.raiseToGround.start(()-> gamepad2.dpad_down);
        //rob.coneManipulator.retractOdom.start(()-> gamepad1.right_stick_button);
        rob.coneManipulator.correctCone.start(()-> gamepad1.circle);
        rob.coneManipulator.raiseLift.start(()-> gamepad1.triangle);
        rob.coneManipulator.releaseOdom.start(()-> gamepad1.left_stick_button);
        rob.coneManipulator.dropConeAndReturnFar.start(()-> gamepad2.left_trigger > 0.1);

        rob.intake.setDirection(Intake.DIRECTION.front);

        double speed = 0.6;

       // rob.coneManipulator.setPosition(ConeManipulator.V4BPreset.PRIME);
        rob.coneManipulator.open();
        boolean front = true;

        while(opModeIsActive() && !isStopRequested()) {

            if(gamepad1.touchpad && timer.currentSeconds() > 0.3) {
                timer.reset();
                gamepad1.rumble(300);
                front = !front;
            }

            if(gamepad2.touchpad && timer.currentSeconds() > 0.3) {
                timer.reset();
                gamepad2.rumble(300);
                rob.coneManipulator.auto = !rob.coneManipulator.auto;
            }

            if(gamepad1.right_bumper) {
                speed = 1;
            }
            if(gamepad1.circle) {
              //  speed = 0.8;
            }
            if(gamepad1.left_bumper) {
                speed = 0.6;
            }
            if(gamepad1.square) {
              //  speed = 0.4;
            }

            if(gamepad1.left_stick_button) {
                rob.release();
            }
            if(gamepad1.right_stick_button) {
                rob.retract();
            }

            if(gamepad1.dpad_down) {
                rob.coneStack = 2;
                rob.coneManipulator.grabConeFromStack.start();
            }
            if(gamepad1.dpad_up) {
                rob.coneStack = 4;
                rob.coneManipulator.grabConeFromStack.start();
            }
            if(gamepad1.dpad_right) {
                rob.coneStack = 5;
                rob.coneManipulator.grabConeFromStack.start();
            }
            if(gamepad1.dpad_left) {
                rob.coneStack = 3;
                rob.coneManipulator.grabConeFromStack.start();
            }

            if(gamepad2.right_stick_button) {
                rob.coneManipulator.resetEnc();
            }

            telemetry.addLine(Color.WHITE.format("---------------MATCH DATA----------------"));
            telemetry.addData(Color.WHITE.format("ROBOT SPEED"), Color.WHITE.format(speed));

            if(front) {
                rob.intake.setDirection(Intake.DIRECTION.back);
                gamepad1.setLedColor(0, 255, 255, 300);
                telemetry.addData("Intake SIDE", Color.CYAN.format("FRONT"));
            }
            else {
                rob.intake.setDirection(Intake.DIRECTION.front);
                gamepad1.setLedColor(255, 255, 0, 300);
                telemetry.addData("Intake SIDE", Color.YELLOW.format("BACK"));
            }

            if(rob.coneManipulator.auto) {
                gamepad2.setLedColor(0, 255, 0, 300);
            }
            else {
                gamepad1.setLedColor(255, 0, 0, 300);
            }

            if(gamepad1.left_trigger > 0.1) {
                rob.intake.setPower(-1);
            }
            else if(gamepad1.right_trigger > 0.1) {
                rob.intake.setPower(1);
            }
            else {
                rob.intake.setPower(0);
            }

            // Auto intake
            if(rob.intake.hasCone() && rob.lift.isLiftDown() && !rob.coneManipulator.grabCone.isActionRunning() && !gamepad2.right_bumper && gamepad2.right_trigger > 0.1) {
                rob.coneManipulator.grabCone.start();
            }

            if(gamepad2.right_bumper) {
                rob.coneManipulator.grabCone.start();
            }

            if(gamepad2.circle) {
                rob.lift.setModeAutomatic();
            }
            if(gamepad2.square) {
                rob.lift.setModeManuel();
            }
            if(gamepad2.cross) {
                rob.lift.lower.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                rob.lift.lower.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            }

            double liftPower = -0.5 * gamepad2.left_stick_y;

            rob.lift.setManuealPower(liftPower);

            if(rob.intake.direction == Intake.DIRECTION.front) {
                rob.driveTrain.setWeightedDrivePower(new Pose2d(-gamepad1.left_stick_y * speed, -gamepad1.left_stick_x * speed, -gamepad1.right_stick_x*0.65));
            }
            else {
                rob.driveTrain.setWeightedDrivePower(new Pose2d(gamepad1.left_stick_y * speed, gamepad1.left_stick_x * speed, -gamepad1.right_stick_x*0.65));

            }

            if(rob.lift.manueal) {
                telemetry.addData("Lift Mode", Color.RED.format("Manual"));
            }
            else {
                telemetry.addData("Lift Mode", Color.GREEN.format("Automatic"));
            }

            if(rob.coneManipulator.auto) {
                telemetry.addData("V4B Mode", Color.GREEN.format("Automatic"));
            }
            else {
                telemetry.addData("V4B Mode", Color.RED.format("Manual"));

            }

            if(rob.intake.hasCone()) {
                telemetry.addLine(Color.GREEN.format("HAS CONE"));
                //telemetry.addData("Detector Distance", Color.GREEN.format(rob.intake.detector.getDistance(DistanceUnit.MM)));
            }
            else {
                telemetry.addLine(Color.RED.format("NO CONE"));
                //telemetry.addData("Detector Distance", Color.RED.format(rob.intake.detector.getDistance(DistanceUnit.MM)));
            }
            telemetry.addData("lift pos", rob.lift.getEncoderPosition());
            telemetry.addData("fourbar pos", rob.coneManipulator.fourbar.getCurrentPosition());
            telemetry.addData("fourbar angle", Math.toDegrees(rob.coneManipulator.getAngle()));
            telemetry.update();

            try {
                rob.update();
            } catch (Exception e) {
                e.printStackTrace();
            }

        }
    }
}