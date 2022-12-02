package org.firstinspires.ftc.teamcode.OpModes;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.ejml.data.DGrowArray;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.ConeManipulator;
import org.firstinspires.ftc.teamcode.util.Color;

@Config
@TeleOp
public class Testing extends LinearOpMode {


    // Intake
    // Lift
    // Minip

    public static double intakeSpeed = 0.0;
    public static double liftSpeed = 0.5;

    public static double left = 1;
    public static double right = 0.09;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        telemetry.setItemSeparator(" >> ");

        Robot rob = new Robot(hardwareMap, telemetry, Robot.OPMODE_TYPE.TELEOP);

        rob.init();

        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {

            if(gamepad1.right_bumper) rob.intake.start();
            else if (gamepad1.left_bumper) rob.intake.reverse();
            else rob.intake.stop();

            rob.driveTrain.setMotorPowers(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);

            double liftPower = (liftSpeed * gamepad1.right_trigger) + (-liftSpeed * gamepad1.left_trigger);

            rob.lift.setPower(liftPower);

            if(gamepad1.dpad_down) {
                rob.coneManipulator.setPosition(ConeManipulator.V4BPreset.IN_MOST);
            }
            if(gamepad1.dpad_left) {
                rob.coneManipulator.setPosition(ConeManipulator.V4BPreset.INNER_PRIME);
            }
            if(gamepad1.dpad_up) {
                rob.coneManipulator.setPosition(ConeManipulator.V4BPreset.Vertiacal);
            }
            if(gamepad1.dpad_right) {
                rob.coneManipulator.setPosition(ConeManipulator.V4BPreset.DROP);
            }

            if(gamepad1.x) {
                rob.coneManipulator.open();
            }
            if(gamepad1.b) {
                rob.coneManipulator.close();
            }

            telemetry.addData("Lift Position", rob.lift.getEncoderPosition());
            telemetry.update();

            try {
                rob.update();
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }
}
