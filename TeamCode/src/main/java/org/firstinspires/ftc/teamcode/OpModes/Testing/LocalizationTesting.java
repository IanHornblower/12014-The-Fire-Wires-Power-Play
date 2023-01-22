
package org.firstinspires.ftc.teamcode.OpModes.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Intake;

@Disabled
@TeleOp(name = "Localization testing")
public class LocalizationTesting extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        telemetry.setItemSeparator(" >> ");

        Robot rob = new Robot(hardwareMap, telemetry, Robot.OPMODE_TYPE.AUTO);

        rob.init();

        waitForStart();

        rob.coneManipulator.retractOdom.start(()-> gamepad1.left_stick_button);
        rob.coneManipulator.releaseOdom.start(()-> gamepad1.right_stick_button);

        rob.intake.setDirection(Intake.DIRECTION.back);


        telemetry = FtcDashboard.getInstance().getTelemetry();

        while(opModeIsActive() && !isStopRequested()) {

            rob.driveTrain.driveFieldCentric(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x*0.8, 0);


            telemetry.addData("left", rob.localizer.getWheelPositions().get(0));
            telemetry.addData("right", rob.localizer.getWheelPositions().get(1));
            telemetry.addData("front", rob.localizer.getWheelPositions().get(2));
            telemetry.addData("pose", rob.localizer.getPose().toString());
            telemetry.update();

            try {
                rob.update();
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }
}
