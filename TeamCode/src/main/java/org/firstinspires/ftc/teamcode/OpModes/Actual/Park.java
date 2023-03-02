package org.firstinspires.ftc.teamcode.OpModes.Actual;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.SuperDuperUsefulStuff.OpModeStuff.OpModeInformations;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Intake;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Lift;
import org.firstinspires.ftc.teamcode.hardware.subsystems.RearCamera;
import org.firstinspires.ftc.teamcode.util.Color;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.Timer;
import org.firstinspires.ftc.teamcode.vision.CombinedTracker;

@Autonomous(name="Just Park", group = "AUTO")
public class Park extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Setup Robot Hardware & Define OpMode Type
        Robot rob = new Robot(hardwareMap, telemetry, Robot.OPMODE_TYPE.AUTO);

        // Fancify Telemetry
        telemetry = rob.getTelemetry();

        // Start Pose
        Pose2d start = new Pose2d(-39, -62, Math.toRadians(-90));
        rob.driveTrain.setPoseEstimate(start);

        // Reset 4bar enc
        rob.coneManipulator.resetEnc();

        // Init Robot
        rob.init();

        // Timer for init & match Timer
        Timer timer = new Timer();
        timer.start();

        if(isStopRequested()) return;

        TrajectorySequence left = rob.driveTrain.trajectorySequenceBuilder(start)
                .strafeTo(new Vector2d(-60, -58))
                .setReversed(true)
                .splineTo(new Vector2d(-60, -17), Math.toRadians(90))
                .build();

        TrajectorySequence middle = rob.driveTrain.trajectorySequenceBuilder(start)
                .setReversed(true)
                .splineTo(new Vector2d(-36, -38), Math.toRadians(90))
                .splineTo(new Vector2d(-36, -20), Math.toRadians(90))
                .build();

        TrajectorySequence right = rob.driveTrain.trajectorySequenceBuilder(start)
                .strafeTo(new Vector2d(-6, -58))
                .setReversed(true)
                .splineTo(new Vector2d(-6, -20), Math.toRadians(90))
                .build();

        rob.release();

        //CombinedTracker.ParkingPosition location = CombinedTracker.ParkingPosition.CENTER;

        RearCamera.State location = RearCamera.State.NONE;


        while(opModeInInit() && !isStopRequested()) {
            telemetry.addLine(Color.WHITE.format("INIT FINISHED")); // Do this later | fancy title
            telemetry.addLine(rob.rearCamera.getTelemetry());
            telemetry.addData("pos", rob.rearCamera.getSleeveLocation().toString());
            telemetry.addData("Time Since Init", Color.LIME.format(timer.currentSeconds()));
            telemetry.update();

            location = rob.rearCamera.getSleeveLocation();

            rob.update();

            if(isStopRequested()) return;
        }

        if(isStopRequested()) return;

        switch (location) {
            case LEFT:
                rob.driveTrain.followTrajectorySequenceAsync(left);
                break;
            case MIDDLE:
                rob.driveTrain.followTrajectorySequenceAsync(middle);
                break;
            case RIGHT:
                rob.driveTrain.followTrajectorySequenceAsync(right);
                break;
        }

        waitForStart();

        rob.rearCamera.camera.stopStreaming();
        rob.rearCamera.camera.closeCameraDevice();

        while(opModeIsActive() && !isStopRequested()) {
            rob.driveTrain.update();
            rob.update();


            if(!rob.driveTrain.isBusy()) {
                rob.driveTrain.setMotorPowers(0, 0,0, 0);
                //stop();
            }

            telemetry.addLine(Color.WHITE.format("---------------AUTO DATA----------------"));
            telemetry.addData("Time Elapsed in Auto", Color.LIME.format(MathUtil.roundPlaces(timer.currentSeconds(), 2)));
            telemetry.addLine();
            telemetry.addData("Robot Position", rob.driveTrain.getPoseEstimate().toString());
            telemetry.addLine();
            telemetry.addData("Lift Encoder State", rob.lift.getEncoderPosition());
            telemetry.addData("Lift Encoder Referance", rob.lift.position);
            telemetry.addLine();
            telemetry.addData("Fourbar Encoder Target", rob.coneManipulator.position);
            telemetry.addData("Fourbar Encoder State", rob.coneManipulator.fourbar.getCurrentPosition());
            telemetry.addData("Fourbar Angle State", Math.toDegrees(rob.coneManipulator.getAngle()));

            telemetry.addLine(Color.WHITE.format("AMOGUS BELOW") + Color.RED.format(" BEWARE"));
            telemetry.addLine();
            telemetry.addLine();
            telemetry.addLine(OpModeInformations.getMongus(timer.currentSeconds()));
            telemetry.update();
        }
    }
}
