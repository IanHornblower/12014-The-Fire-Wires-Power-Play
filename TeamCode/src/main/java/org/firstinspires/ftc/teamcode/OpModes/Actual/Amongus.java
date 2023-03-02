
package org.firstinspires.ftc.teamcode.OpModes.Actual;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.SuperDuperUsefulStuff.OpModeStuff.OpModeInformations;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Intake;
import org.firstinspires.ftc.teamcode.util.Color;
import org.firstinspires.ftc.teamcode.util.Timer;

@Config
@TeleOp(name = "Amongus Twerk Testing (AT&T)")
public class Amongus extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        telemetry.setMsTransmissionInterval(80);

        Robot rob = new Robot(hardwareMap, telemetry, Robot.OPMODE_TYPE.DASHBOARD_TESTING);

        telemetry = rob.getTelemetry();

        Timer timer = new Timer();
        timer.reset();

        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {
            rob.update();
            telemetry.addLine(Color.WHITE.format("AMOGUS BELOW") + Color.RED.format(" BEWARE"));
            telemetry.addLine();
            telemetry.addLine();
            //telemetry.addLine(OpModeInformations.testFinal);
            telemetry.addLine(Color.YELLOW.format(rob.mogus.get38pxTwerk()));
            //telemetry.addLine(OpModeInformations.testFinal);
            telemetry.update();
        }


    }
}