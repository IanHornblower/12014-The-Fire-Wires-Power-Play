package org.firstinspires.ftc.teamcode.OpModes.Actual;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ActionSystem.ActionSequence;
import org.firstinspires.ftc.teamcode.ActionSystem.ActionSequenceRunner;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.util.Color;
import org.firstinspires.ftc.teamcode.util.Timer;

@Disabled
@Autonomous
public class RightSideAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Setup Robot Hardware & Define OpMode Type
        Robot rob = new Robot(hardwareMap, telemetry, Robot.OPMODE_TYPE.AUTO);

        // Fancify Telemetry
        telemetry = rob.getTelemetry();

        // Init Robot
        rob.init();

        // Timer for init & match Timer
        Timer timer = new Timer();
        timer.start();

        ActionSequence lift = new ActionSequence();
        ActionSequence driveTrain = new ActionSequence(rob);

        ActionSequenceRunner driveTrainRunner = new ActionSequenceRunner(rob);
        driveTrainRunner.setActionSequence(driveTrain);

        ActionSequenceRunner liftRunner = new ActionSequenceRunner(rob);
        liftRunner.setActionSequence(lift);

        if(isStopRequested()) return;

        driveTrain.FollowTrajectory(rob.driveTrain.trajectoryBuilder(rob.driveTrain.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(0, 0, 0))
                .build()
        );

        while(opModeInInit() && !isStopRequested()) {
            telemetry.addLine(Color.WHITE.format("INIT FINISHED")); // Do this later | fancy title
            telemetry.addData("Time Since Init", Color.LIME.format(timer.currentSeconds()));
            telemetry.update();

            switch (rob.rearCamera.position) {
                case LEFT:
                    break;
                case CENTER:
                    break;
                case RIGHT:
                    break;
            }

            if(isStopRequested()) return;
        }

        if(isStopRequested()) return;

        waitForStart();

        rob.rearCamera.camera.stopStreaming();
        rob.rearCamera.camera.closeCameraDevice();

        while(opModeIsActive() && !isStopRequested()) {
            if(!driveTrainRunner.isComplete()) {
                try {
                    driveTrainRunner.update();
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }

            if(!liftRunner.isComplete()) {
                try {
                    liftRunner.update();
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }


            if(driveTrainRunner.isComplete() && liftRunner.isComplete()) {
                rob.driveTrain.setMotorPowers(0, 0,0, 0);
                stop();
            }
            
            telemetry.update();
        }
    }
}
