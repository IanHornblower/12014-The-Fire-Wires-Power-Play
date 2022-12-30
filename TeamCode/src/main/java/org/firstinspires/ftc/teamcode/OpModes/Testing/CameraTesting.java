package org.firstinspires.ftc.teamcode.OpModes.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.stream.CameraStreamServer;
import org.firstinspires.ftc.teamcode.hardware.Robot;

@Config
@TeleOp
public class CameraTesting extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot rob = new Robot(hardwareMap, telemetry, Robot.OPMODE_TYPE.AUTO);
        rob.init();

        rob.FtcDashboardInstance.startCameraStream(rob.rearCamera.camera, 0);
        CameraStreamServer.getInstance().setSource(rob.rearCamera.camera);

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
