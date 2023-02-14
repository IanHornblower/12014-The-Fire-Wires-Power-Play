package org.firstinspires.ftc.teamcode.OpModes.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.stream.CameraStreamServer;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.util.Color;
import org.firstinspires.ftc.teamcode.vision.CombinedTracker;

@Config
@TeleOp
public class CameraTuning extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot rob = new Robot(hardwareMap, telemetry, Robot.OPMODE_TYPE.AUTO);
        rob.init();

        rob.FtcDashboardInstance.startCameraStream(rob.rearCamera.camera, 0);
        CameraStreamServer.getInstance().setSource(rob.rearCamera.camera);

        while(opModeInInit() && !isStopRequested()) {
            telemetry.addLine(Color.WHITE.format("INIT FINISHED")); // Do this later | fancy title
            telemetry.addLine(rob.rearCamera.getTelemetry());
            telemetry.addData("pos", rob.rearCamera.getSleeveLocation().toString());
            //telemetry.addData("Time Since Init", Color.LIME.format(timer.currentSeconds()));



            if(gamepad1.dpad_left && !(gamepad1.left_trigger > 0.1)) {
                CombinedTracker.anchorPointX -= 1;
            }

            if(gamepad1.dpad_right && !(gamepad1.left_trigger > 0.1)) {
                CombinedTracker.anchorPointX += 1;
            }

            if(gamepad1.dpad_down && !(gamepad1.left_trigger > 0.1)) {
                CombinedTracker.anchorPointY -= 1;
            }

            if(gamepad1.dpad_up && !(gamepad1.left_trigger > 0.1)) {
                CombinedTracker.anchorPointY += 1;
            }


            if(gamepad1.dpad_left && (gamepad1.left_trigger > 0.1)) {
                CombinedTracker.anchorPointX -= 5;
            }

            if(gamepad1.dpad_right && (gamepad1.left_trigger > 0.1)) {
                CombinedTracker.anchorPointX += 5;
            }

            if(gamepad1.dpad_down && (gamepad1.left_trigger > 0.1)) {
                CombinedTracker.anchorPointY -= 5;
            }

            if(gamepad1.dpad_up && (gamepad1.left_trigger > 0.1)) {
                CombinedTracker.anchorPointY += 5;
            }

            if(isStopRequested()) return;

            rob.rearCamera.reInit();

            telemetry.addData("point x", CombinedTracker.anchorPointX);
            telemetry.addData("point y", CombinedTracker.anchorPointY);
            telemetry.update();
        }

        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {


            telemetry.addData("Cone Error", rob.rearCamera.getObjectError());

            try {
                rob.update();
            } catch (Exception e) {
                e.printStackTrace();
            }
            telemetry.update();
        }
    }
}
