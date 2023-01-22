
package org.firstinspires.ftc.teamcode.OpModes.Actual;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
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
@TeleOp(name = "TeleOp")
public class NewTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        telemetry.setItemSeparator(" >> ");

        Robot rob = new Robot(hardwareMap, telemetry, Robot.OPMODE_TYPE.TELEOP);

        telemetry = rob.getTelemetry();

        rob.init();

        Timer timer = new Timer();
        timer.reset();

        rob.tempSideFlip = ()-> gamepad2.left_trigger > 0.1;

        waitForStart();

        rob.coneManipulator.raiseToTop.start(()-> gamepad2.dpad_right); // GROUND
        rob.coneManipulator.raiseToMid.start(()-> gamepad2.dpad_up); // GROUND
        rob.coneManipulator.raiseToLow.start(()-> gamepad2.dpad_left); // GROUND
        rob.coneManipulator.dropConeAndReturn.start(()-> gamepad2.left_bumper); // GROUND
        rob.coneManipulator.grabCone.start(()-> gamepad2.right_bumper);
        rob.coneManipulator.raiseToGround.start(()-> gamepad2.dpad_down);
        rob.coneManipulator.retractOdom.start(()-> gamepad2.left_stick_button);
        rob.coneManipulator.releaseOdom.start(()-> gamepad2.right_stick_button);

        rob.intake.setDirection(Intake.DIRECTION.front);

        double speed = 0.6;

       // rob.coneManipulator.setPosition(ConeManipulator.V4BPreset.PRIME);
        rob.coneManipulator.open();
        boolean front = true;

        while(opModeIsActive() && !isStopRequested()) {

            if(gamepad2.touchpad && timer.currentSeconds() > 0.3) {
                timer.reset();
                gamepad2.rumble(300);
                front = !front;
            }

            if(gamepad1.triangle) {
                speed = 1;
            }
            if(gamepad1.circle) {
                speed = 0.8;
            }
            if(gamepad1.cross) {
                speed = 0.6;
            }
            if(gamepad1.square) {
                speed = 0.4;
            }

            telemetry.addLine(Color.WHITE.format("---------------MATCH DATA----------------"));
            telemetry.addData(Color.WHITE.format("ROBOT SPEED"), Color.WHITE.format(speed));

            if(front) {
                rob.intake.setDirection(Intake.DIRECTION.back);
                gamepad2.setLedColor(0, 255, 255, 300);
                telemetry.addData("Intake SIDE", Color.CYAN.format("BACK"));
            }
            else {
                rob.intake.setDirection(Intake.DIRECTION.front);
                gamepad2.setLedColor(255, 255, 0, 300);
                telemetry.addData("Intake SIDE", Color.YELLOW.format("FRONT"));
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

            if(rob.intake.hasCone() && rob.lift.isLiftDown() && !rob.coneManipulator.grabCone.isActionRunning() && !gamepad2.right_bumper) {
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

            rob.driveTrain.setWeightedDrivePower(gamepad1.left_stick_x * speed, -gamepad1.left_stick_y * speed, gamepad1.right_stick_x*0.65);


            if(rob.lift.manueal) {
                telemetry.addData("Lift Mode", Color.RED.format("Manuel"));
            }
            else {
                telemetry.addData("Lift Mode", Color.GREEN.format("Automatic"));
            }


            //telemetry.addData("Amps",rob.lift.lower.getCurrent(CurrentUnit.MILLIAMPS));
            //telemetry.addData("lift enc", rob.lift.getEncoderPosition());
            if(rob.intake.hasCone()) {
                telemetry.addLine(Color.GREEN.format("HAS CONE"));
                //telemetry.addData("Detector Distance", Color.GREEN.format(rob.intake.detector.getDistance(DistanceUnit.MM)));
            }
            else {
                telemetry.addLine(Color.RED.format("NO CONE"));
                //telemetry.addData("Detector Distance", Color.RED.format(rob.intake.detector.getDistance(DistanceUnit.MM)));
            }
            telemetry.addData("lift pos", rob.lift.getEncoderPosition());
            telemetry.update();

            try {
                rob.update();
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }
}
