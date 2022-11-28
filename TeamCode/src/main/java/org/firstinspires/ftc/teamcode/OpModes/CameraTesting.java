package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamServer;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.util.Color;

//@Config
//@TeleOp
@Disabled
public class CameraTesting extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        telemetry.setItemSeparator(" >> ");

        Robot rob = new Robot(hardwareMap, Robot.OPMODE_TYPE.AUTO);

        FtcDashboard.getInstance().startCameraStream(rob.sleeveDetectionCamera.camera, 0);
        CameraStreamServer.getInstance().setSource(rob.sleeveDetectionCamera.camera);

        rob.init();

        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {

            telemetry.addLine(rob.sleeveDetectionCamera.getTelemetry());

            try {
                rob.update();
            } catch (Exception e) {
                e.printStackTrace();
            }
            telemetry.update();
        }
    }
}
