package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.util.Color;

@Config
@TeleOp
public class CameraTesting extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        Robot rob = new Robot(hardwareMap, telemetry);

        rob.init();

        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {

            rob.telemetry.addLine(rob.sleeveDetectionCamera.getTelemetry());

            rob.update();
            telemetry.update();
        }
    }
}
