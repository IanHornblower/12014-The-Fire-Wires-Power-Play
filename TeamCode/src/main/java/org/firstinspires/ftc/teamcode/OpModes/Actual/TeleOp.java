package org.firstinspires.ftc.teamcode.OpModes.Actual;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.util.Color;

@Config
@com.qualcomm.robotcore.eventloop.opmode.TeleOp
public class TeleOp extends LinearOpMode {

    public static double intakeSpeed = 0.0;
    public static double liftSpeed = 0.5;

    public static double left = 1;
    public static double right = 0.09;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        telemetry.setItemSeparator(" >> ");

        Robot rob = new Robot(hardwareMap, telemetry);


        rob.init();

        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {

            rob.intake.setPower(intakeSpeed);

            rob.driveTrain.setMotorPowers(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);

            double liftPower = (liftSpeed * gamepad1.right_trigger) + (-liftSpeed * gamepad1.left_trigger);

            rob.lift.setPower(liftPower);


            // Make this Color Formatting System Simpler

            if(rob.lift.getEncoderPosition() < 0 || rob.lift.getEncoderPosition() > 200) {
                telemetry.addData("Lift Position", Color.RED.format(rob.lift.getEncoderPosition()));
            }
            else {
                telemetry.addData("Lift Position", Color.GREEN.format(rob.lift.getEncoderPosition()));
            }


            rob.coneManipulator.setLeft(left);
            rob.coneManipulator.setRight(right);

            telemetry.update();

            rob.update();
        }
    }
}
