package org.firstinspires.ftc.teamcode.OpModes.Actual;

import android.util.Base64DataException;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.ejml.data.DGrowArray;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ActionSystem.ActionSequence;
import org.firstinspires.ftc.teamcode.ActionSystem.ActionSequenceRunner;
import org.firstinspires.ftc.teamcode.ActionSystem.actions.CustomAction;
import org.firstinspires.ftc.teamcode.ActionSystem.actions.Wait;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.util.Color;
import org.firstinspires.ftc.teamcode.util.ColorTelemetry;
import org.firstinspires.ftc.teamcode.util.Timer;
import org.firstinspires.ftc.teamcode.vision.CombinedTracker;

import java.util.ArrayList;

@Disabled
@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp(name = "Old TeleOp")
public class TeleOp extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Setup Robot Hardware & Define OpMode Type
        Robot rob = new Robot(hardwareMap, telemetry, Robot.OPMODE_TYPE.TELEOP);

        // Fancify Telemetry
        telemetry = rob.getTelemetry();

        // Init Robot
        rob.init();

        // Timer for init & match Timer
        Timer timer = new Timer();
        timer.start();

        // Timer to switch between operator modes
        Timer profileTimer = new Timer();
        profileTimer.start();
        boolean manualMode = false;

        Timer autoGrabTimer = new Timer();
        autoGrabTimer.start();


        Timer driveSpeedSelectTimer = new Timer();
        driveSpeedSelectTimer.start();
        double speed = 0.6;

        rob.rearCamera.combinedTracker.setTrackType(CombinedTracker.TrackType.POLE);
        rob.rearCamera.combinedTracker.horizon = 40;

        while(opModeInInit() && !isStopRequested()) {
            telemetry.addLine(); // Do this later | fancy title
            telemetry.addData("Time Since Init", Color.LIME.format(timer.currentSeconds()));
            telemetry.update();
        }

        rob.coneManipulator.open();
        //rob.coneManipulator.setPosition(ConeManipulator.V4BPreset.INNER_PRIME);

        while(opModeIsActive() && !isStopRequested()) {

            if(gamepad2.touchpad && profileTimer.currentSeconds() > 0.3) {
                profileTimer.reset();
                gamepad2.rumble(300);
                manualMode = !manualMode;
            }

            if(manualMode) { // Put Manual shit
                telemetry.addData("Operating Mode", Color.RED.format("Manual"));
                gamepad2.setLedColor(255.0, 0.0, 0.0, 5000);

                // Use DPad for setting lift position
                /*
                if(gamepad2.dpad_down) {
                    rob.coneManipulator.setPosition(ConeManipulator.V4BPreset.IN_MOST);
                }
                if(gamepad2.dpad_left) {
                    rob.coneManipulator.setPosition(ConeManipulator.V4BPreset.INNER_PRIME);
                }
                if(gamepad2.dpad_up) {
                    rob.coneManipulator.setPosition(ConeManipulator.V4BPreset.Vertiacal);
                }
                if(gamepad2.dpad_right) {
                    rob.coneManipulator.setPosition(ConeManipulator.V4BPreset.DROP);
                }

                // Close and Open the Claw
                // X/Square -> Open
                // B/Circle -> Close
                if(gamepad2.x) {
                    rob.coneManipulator.open();
                }
                if(gamepad2.b) {
                    rob.coneManipulator.close();
                }

                 */





            }
            else {
                // Put Automatic shit
                // Auto Grab Cone when thingy is there

                telemetry.addData("Operating Mode", Color.GREEN.format("Automatic"));
                gamepad2.setLedColor(0.0, 255.0, 0.0, 5000);

                // Automatically Grab Cone if it is in the proper position

                if(rob.intake.hasCone() && rob.lift.isLiftDown() && !rob.coneManipulator.grabCone.isActionRunning() && !gamepad2.right_bumper) {
                    rob.coneManipulator.grabCone.start();
                }

                rob.coneManipulator.raiseToGround.start(()-> gamepad2.dpad_down); // GROUND

                rob.coneManipulator.raiseToTop.start(()-> gamepad2.dpad_right); // TOP

                rob.coneManipulator.raiseToMid.start(()-> gamepad2.dpad_up); // MID

                rob.coneManipulator.raiseToLow.start(()-> gamepad2.dpad_left); // LOW

                double liftPower = (1 * gamepad2.right_trigger) + (-0.7 * gamepad2.left_trigger);

                if(!rob.coneManipulator.dropConeAndReturn.isActionRunning() ||
                        !rob.coneManipulator.raiseToLow.isActionRunning() ||
                        !rob.coneManipulator.raiseToMid.isActionRunning() ||
                        !rob.coneManipulator.raiseToTop.isActionRunning() ||
                        gamepad2.right_trigger + gamepad2.left_trigger > 0.1
                ) {
                    //rob.lift.setPower(liftPower);
                }

                // Right Bumper - Grab and Prime Cone
                if(gamepad2.right_bumper) {
                    rob.coneManipulator.grabCone.start();
                }

                // Left Bumper - Release and Lower lift
                rob.coneManipulator.dropConeAndReturn.start(()-> gamepad2.left_bumper);

            }

            // Lift - Gamepad 2

            // Con
            // RIGHT - 4
            // Lat - 5

            // Ex
            // left - 1/0






            /*
             * Game Pad 1 - Driver
             */

            /*
             * Intake - Gamepad 1
             *
             * Control Schema
             *      - Left Trigger - Reversed Intake
             *      - Right Trigger - Regular Intake
             *      - When not pressed - Is Stopped
             */

            if(gamepad1.left_bumper && driveSpeedSelectTimer.currentSeconds() > 0.2) {
                speed -= 0.2;
                driveSpeedSelectTimer.reset();
            }

            if(gamepad1.right_bumper && driveSpeedSelectTimer.currentSeconds() > 0.2) {
                speed += 0.2;
                driveSpeedSelectTimer.reset();
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

            if(gamepad1.right_trigger > 0.1) rob.intake.start();
            else if (gamepad1.left_trigger > 0.1) rob.intake.reverse();
            else rob.intake.stop();

            /*
             * Drive Train - Gamepad 1
             *
             * Control Schema
             *      - Left Joystick - Control Forward/Backward & Strafe Left/Right
             *      - When not pressed - Do not move
             */

           // rob.driveTrain.setWeightedDrivePower(gamepad1.left_stick_x*speed, -gamepad1.left_stick_y*speed, gamepad1.right_stick_x*0.8);
            //rob.driveTrain.driveFieldCentric(gamepad1.left_stick_x*speed, -gamepad1.left_stick_y*speed, gamepad1.right_stick_x*speed);

            telemetry.addLine(Color.WHITE.format("---------------MATCH DATA----------------"));
            telemetry.addData(Color.WHITE.format("ROBOT SPEED"), Color.WHITE.format(speed));
            if(rob.intake.hasCone()) {
                telemetry.addLine(Color.GREEN.format("HAS CONE"));
                //telemetry.addData("Detector Distance", Color.GREEN.format(rob.intake.detector.getDistance(DistanceUnit.MM)));
            }
            else {
                telemetry.addLine(Color.RED.format("NO CONE"));
                //telemetry.addData("Detector Distance", Color.RED.format(rob.intake.detector.getDistance(DistanceUnit.MM)));
            }

            telemetry.addLine(Color.WHITE.format("---------------DEBUG----------------"));

            telemetry.addData("Lift Power", rob.lift.lower.getPower());
            telemetry.addData("Limit Switch", rob.lift.isLiftDown());
            telemetry.addData("Lift Encoder", rob.lift.getEncoderPosition());
         //   telemetry.addData("Front Left Value", rob.driveTrain.motors[0].getCurrentPosition());
            telemetry.addData("imu velo", Math.toDegrees(rob.imu.getExternalHeadingVelocity()));

           // telemetry.addData("Distacne", rob.lineAlignment.getDistance());
           // telemetry.addData("left color", rob.lineAlignment.getLeft());
           // telemetry.addData("right color", rob.lineAlignment.getRight());
            telemetry.update();

            // Robot Update Call

            try {
                rob.update();
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }
}
