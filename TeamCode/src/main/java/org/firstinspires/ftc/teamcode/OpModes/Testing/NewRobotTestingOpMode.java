
package org.firstinspires.ftc.teamcode.OpModes.Testing;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.ConeManipulator;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Intake;

@Config
@TeleOp(name = "New Robot Testing")
public class NewRobotTestingOpMode extends LinearOpMode {


    // Intake
    // Lift
    // Minip

    public static double intakeSpeed = 0.0;
    public static double liftSpeed = 1;

    public static double liftPosition = 0;

    public static double left = 1;
    public static double right = 0.00;
    public static double upperPower = 0;
    public static double lowerPower = 0;

    @Override
    public void runOpMode() throws InterruptedException {
        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        telemetry.setItemSeparator(" >> ");

        Robot rob = new Robot(hardwareMap, telemetry, Robot.OPMODE_TYPE.AUTO);

        rob.init();

        waitForStart();

        rob.coneManipulator.raiseToTop.start(()-> gamepad2.dpad_right); // GROUND
        rob.coneManipulator.raiseToMid.start(()-> gamepad2.dpad_up); // GROUND
        rob.coneManipulator.raiseToLow.start(()-> gamepad2.dpad_left); // GROUND
        rob.coneManipulator.dropConeAndReturn.start(()-> gamepad2.left_bumper); // GROUND
        rob.coneManipulator.grabCone.start(()-> gamepad2.right_bumper);
        rob.coneManipulator.retractOdom.start(()-> gamepad2.left_stick_button);
        rob.coneManipulator.releaseOdom.start(()-> gamepad2.right_stick_button);

        rob.intake.setDirection(Intake.DIRECTION.back);

        while(opModeIsActive() && !isStopRequested()) {

            if(gamepad1.left_bumper) {
                rob.intake.setPower(-1);
            }
            else if(gamepad1.right_bumper) {
                rob.intake.setPower(1);
            }
            else {
                rob.intake.setPower(0);
            }

            double liftPower = (liftSpeed * gamepad1.right_trigger) + (-liftSpeed * gamepad1.left_trigger);

            rob.driveTrain.setWeightedDrivePower(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x*0.8);


            //rob.localizer.leftRetract.setPower(intakeSpeed);
            //rob.localizer.rightRetract.setPower(intakeSpeed);



            //rob.lift.setPower(liftPower);
/*

            if(gamepad1.cross) {
                rob.coneManipulator.close();
            }
            if(gamepad1.circle) {
                rob.coneManipulator.open();
            }


 */

            rob.coneManipulator.setServoPositions(new double[] {left, right});

            //rob.lift.lower.setPower(lowerPower);
            //rob.lift.upper.setPower(upperPower);




            telemetry.addData("Amps",rob.lift.lower.getCurrent(CurrentUnit.MILLIAMPS));
            telemetry.addData("lift enc", rob.lift.getEncoderPosition());
            telemetry.addData("lift power", liftPower);
            telemetry.update();

            try {
                rob.update();
            } catch (Exception e) {
                e.printStackTrace();
            }
        }
    }
}
