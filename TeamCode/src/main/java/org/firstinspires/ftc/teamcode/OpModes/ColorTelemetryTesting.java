package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.util.Color;
import org.firstinspires.ftc.teamcode.util.ColorTelemetry;

@Config
@TeleOp
public class ColorTelemetryTesting extends LinearOpMode {

    public static String text = "Text";

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);

        MultipleTelemetry m_telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        waitForStart();

        Color color = Color.CYAN;

        while(opModeIsActive() && !isStopRequested()) {
            m_telemetry.addData(Color.RED.format("Joe Biden"), ColorTelemetry.getFontFormatted(text, 12, color));
            m_telemetry.update();

        }
    }
}
