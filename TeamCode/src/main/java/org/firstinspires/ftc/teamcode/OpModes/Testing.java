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
public class Testing extends LinearOpMode {


    // Intake
    // Lift
    // Minip

    public static double intakeSpeed = 0.0;
    public static double liftSpeed = 0.5;

    public static double left = 0;
    public static double right = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot rob = new Robot(hardwareMap, telemetry);

        ServoImplEx leftS = hardwareMap.get(ServoImplEx.class, "leftServo");
        ServoImplEx rightS = hardwareMap.get(ServoImplEx.class, "rightServo");

        leftS.setPwmRange(new PwmControl.PwmRange(500, 2500));
        rightS.setPwmRange(new PwmControl.PwmRange(500, 2500));

        rob.init();

        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {

            rob.intake.setPower(intakeSpeed);

            rob.lift.setPower(liftSpeed * gamepad1.right_trigger);

            rob.lift.setPower(-liftSpeed * gamepad1.left_trigger);


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
