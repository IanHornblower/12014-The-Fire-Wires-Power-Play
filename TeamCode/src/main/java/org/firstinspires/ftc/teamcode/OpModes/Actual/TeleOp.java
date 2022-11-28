package org.firstinspires.ftc.teamcode.OpModes.Actual;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
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
import org.firstinspires.ftc.teamcode.hardware.subsystems.ConeManipulator;
import org.firstinspires.ftc.teamcode.util.Color;
import org.firstinspires.ftc.teamcode.util.Timer;

import java.util.ArrayList;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {
    /**
     * Finished - Intake
     * In progress - Lift, V4B
     * Un-started - Jimmy, Pole Detection
     */



    boolean autoGrabbing = false;

    public static double position = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        // Make Telemetry Fancy
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // Setup Robot Hardware & Define OpMode Type
        Robot rob = new Robot(hardwareMap, Robot.OPMODE_TYPE.TELEOP);

        // Init Robot
        rob.init();

        // Timer for init & match Timer
        Timer timer = new Timer();
        timer.start();

        // Timer to switch between operator modes
        Timer profileTimer = new Timer();
        profileTimer.start();
        boolean manualMode = true;

        ActionSequenceRunner coneMinip = new ActionSequenceRunner(rob);

        while(opModeInInit() && !isStopRequested()) {
            telemetry.addLine(); // Do this later | fancy title
            telemetry.addData("Time Since Init", Color.LIME.format(timer.currentSeconds()));
            telemetry.update();
        }

        while(opModeIsActive() && !isStopRequested()) {

            if(gamepad2.touchpad && profileTimer.currentSeconds() > 0.3) {
                manualMode = !manualMode;
            }

            if(manualMode) { // Put Manual shit
                telemetry.addData("Operating Mode", Color.RED.format("Manual"));
                gamepad2.setLedColor(255.0, 0.0, 0.0, 1);
            }
            else {           // Put Automatic shit
                // Auto Grab Cone when thingy is there
                if(rob.intake.hasCone() && !autoGrabbing) {
                    rob.coneManipulator.grabCone.start();
                    autoGrabbing = true;
                }
                if(rob.coneManipulator.grabCone.isComplete()) {
                    autoGrabbing = false;
                }

                telemetry.addData("Operating Mode", Color.BLUE.format("Automatic"));
                gamepad2.setLedColor(0.0, 0.0, 255.0, 1);
            }

            /*
             * Intake - Gamepad 1
             *
             * Control Schema (Change to triggers later || to allow for triggers for V4B or Claw)
             *      - Left Bumper - Reversed Intake
             *      - Right Bumper - Regular Intake
             *      - When not pressed - Is Stopped
             */

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

            rob.driveTrain.setWeightedDrivePower(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);

            // Lift - Gamepad 2

            double liftPower = (0.7 * gamepad2.right_trigger) + (-0.7 * gamepad2.left_trigger);

            //rob.lift.setPower(liftPower);

            rob.lift.runToPosition(position);









            // V4B

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

            // Claw

            if(gamepad2.x) {
                rob.coneManipulator.open();
            }
            if(gamepad2.b) {
                rob.coneManipulator.close();
            }

            rob.coneManipulator.grabCone.start(()-> gamepad1.right_bumper);

            // Telemetry

            telemetry.addData("Lift Encoder", rob.lift.getEncoderPosition());
            telemetry.addData("Detector Distance", rob.intake.detector.getDistance(DistanceUnit.MM));
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
