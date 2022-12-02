package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamServer;
import org.firstinspires.ftc.teamcode.hardware.Robot;

//@Config
//@TeleOp
@Disabled
public class CameraTesting extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        telemetry.setItemSeparator(" >> ");

        Robot rob = new Robot(hardwareMap, telemetry, Robot.OPMODE_TYPE.AUTO);

        FtcDashboard.getInstance().startCameraStream(rob.rearCamera.camera, 0);
        CameraStreamServer.getInstance().setSource(rob.rearCamera.camera);

        rob.init();

        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {

            telemetry.addLine(rob.rearCamera.getTelemetry());

            try {
                rob.update();
            } catch (Exception e) {
                e.printStackTrace();
            }
            telemetry.update();
        }
    }
}
