package org.firstinspires.ftc.teamcode.OpModes.Actual;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ActionSystem.ActionSequence;
import org.firstinspires.ftc.teamcode.ActionSystem.ActionSequenceRunner;
import org.firstinspires.ftc.teamcode.ActionSystem.actions.BasicTurn;
import org.firstinspires.ftc.teamcode.ActionSystem.actions.CustomAction;
import org.firstinspires.ftc.teamcode.ActionSystem.actions.EncoderDrive;
import org.firstinspires.ftc.teamcode.ActionSystem.actions.ReturnToDistance;
import org.firstinspires.ftc.teamcode.ActionSystem.actions.LiftSetPosition;
import org.firstinspires.ftc.teamcode.ActionSystem.actions.WaitFor;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.ConeManipulator;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Lift;
import org.firstinspires.ftc.teamcode.util.Color;
import org.firstinspires.ftc.teamcode.util.Timer;
import org.firstinspires.ftc.teamcode.vision.CombinedTracker;
import org.firstinspires.ftc.teamcode.vision.SleeveDetection;

@Autonomous
public class BlueAutoPark extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Setup Robot Hardware & Define OpMode Type
        Robot rob = new Robot(hardwareMap, telemetry, Robot.OPMODE_TYPE.AUTO);
        //rob.setStartPosition(new Pose2D(0, 0, 0));

        // Fancify Telemetry
        telemetry = rob.getTelemetry();

        // Init Robot
        rob.init();
        //rob.setStartPosition(new Pose2D(0, 0, 0));

        // Timer for init & match Timer
        Timer timer = new Timer();
        timer.start();

        ActionSequence lift = new ActionSequence();
        ActionSequence claw = new ActionSequence();

        ActionSequence driveTrain = new ActionSequence();

        ActionSequenceRunner driveTrainRunner = new ActionSequenceRunner(rob);
        driveTrainRunner.setActionSequence(driveTrain);

        ActionSequenceRunner liftRunner = new ActionSequenceRunner(rob);
        liftRunner.setActionSequence(lift);

        ActionSequenceRunner clawRunner = new ActionSequenceRunner(rob);
        clawRunner.setActionSequence(claw);

        driveTrain.addAction(new EncoderDrive(rob, 0, -0.5, 0, 1450));
        //driveTrain.addAction(new EncoderDrive(rob, 0.0, -0.2, 0, 150));
        //driveTrain.addAction(new EncoderDrive(rob, 0.0, 0.2, 0, 150));
        driveTrain.addAction(new BasicTurn(rob, Math.toRadians(-55)));
        driveTrain.addAction(new EncoderDrive(rob, 0, -0.3, Math.toRadians(-60), 100));

        // Go To Cycle
        driveTrain.addAction(new WaitFor(()-> clawRunner.isActionComplete(6)));
        driveTrain.addAction(new EncoderDrive(rob, 0, 0.3, 0, 320));

        driveTrain.addAction(new BasicTurn(rob, Math.toRadians(90)));
        driveTrain.addWait(0.5);
        driveTrain.addAction(new ReturnToDistance(rob, 0.001));
        driveTrain.addAction(new WaitFor(()-> clawRunner.isActionComplete(15)));
        driveTrain.addWait(0.5);
        driveTrain.addAction(new BasicTurn(rob, Math.toRadians(90)));
        driveTrain.addAction(new EncoderDrive(rob, 0, 0.5, 0, 10));
        driveTrain.addAction(new EncoderDrive(rob, 0, 0.5, 0, 990));
        driveTrain.addAction(new BasicTurn(rob, Math.toRadians(35)));
        driveTrain.addAction(new EncoderDrive(rob, 0, -0.3, 0, 160));

        driveTrain.addAction(new WaitFor(()-> clawRunner.isActionComplete(22)));
        driveTrain.addAction(new BasicTurn(rob, Math.toRadians(90)));
        driveTrain.addAction(new EncoderDrive(rob, -0.3, 0, 0, 150));
        driveTrain.addAction(new BasicTurn(rob, Math.toRadians(90)));

        lift.addAction(new LiftSetPosition(rob, Lift.highPole + 100));
        lift.addAction(new WaitFor(()-> clawRunner.isActionComplete(16)));
        lift.addAction(new LiftSetPosition(rob, Lift.highPole + 100));

        claw.addCustomAction(()-> rob.coneManipulator.close());
        claw.addCustomAction(()-> rob.coneManipulator.setPosition(ConeManipulator.V4BPreset.DROP_AUTO));
        claw.addAction(new WaitFor(()-> driveTrainRunner.isActionComplete(2)));
        claw.addWait(0.5); // Was 0.5
        claw.addAction(new CustomAction(()-> rob.coneManipulator.open()));

        claw.addWait(0.5); // Was 0.5

        claw.addAction(new CustomAction(()-> rob.coneManipulator.setPosition(ConeManipulator.V4BPreset.INNER_PRIME)));
        claw.addAction(new WaitFor(()-> driveTrainRunner.isActionComplete(5)));
        claw.addAction(new CustomAction(()-> rob.coneManipulator.open()));
        claw.addAction(new WaitFor(()-> driveTrainRunner.isActionComplete(6)));

        claw.addAction(new CustomAction(()-> rob.coneManipulator.setPosition(ConeManipulator.V4BPreset.GROUND_LEVEL)));

        claw.addAction(new LiftSetPosition(rob, Lift.smallPole-50));
        claw.addWait(1.2);
        claw.addAction(new CustomAction(()-> rob.coneManipulator.close()));
        claw.addWait(0.3); // Was 0.3

        claw.addAction(new CustomAction(()-> rob.coneManipulator.setPosition(ConeManipulator.V4BPreset.INNER_PRIME)));
        claw.addWait(1);

        claw.addCustomAction(()-> rob.coneManipulator.setPosition(ConeManipulator.V4BPreset.DROP_AUTO));
        claw.addAction(new WaitFor(()-> driveTrainRunner.isActionComplete(13)));
        claw.addWait(0.5); // Was 0.5

        claw.addAction(new CustomAction(()-> rob.coneManipulator.open()));
        claw.addWait(1); // Was 1
        claw.addAction(new CustomAction(()-> rob.coneManipulator.setPosition(ConeManipulator.V4BPreset.INNER_PRIME)));
        claw.addAction(new LiftSetPosition(rob, Lift.lowResting));

        rob.coneManipulator.close();

        while(opModeInInit() && !isStopRequested()) {
            telemetry.addLine(Color.WHITE.format("INIT FINISHED")); // Do this later | fancy title
            telemetry.addLine(rob.rearCamera.getTelemetry());
            telemetry.addData("pos", rob.rearCamera.getSleeveLocation().toString());
            telemetry.addData("Time Since Init", Color.LIME.format(timer.currentSeconds()));
            telemetry.update();
        }

        switch (rob.rearCamera.getSleeveLocation()) {
            case LEFT:
                driveTrain.addAction(new EncoderDrive(rob, 0, 0.3, 0, 60 ));
                break;
            case CENTER:
                driveTrain.addAction(new EncoderDrive(rob, 0, -0.4, 0, 580));
                break;
            case RIGHT:
                driveTrain.addAction(new EncoderDrive(rob, 0, -0.4, 0, 1320));
                break;
        }

        driveTrain.addAction(new BasicTurn(rob, Math.toRadians(90)));

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

            if(!clawRunner.isComplete()) {
                try {
                    clawRunner.update();
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }


            if(driveTrainRunner.isComplete() && liftRunner.isComplete() && clawRunner.isComplete()) {
                rob.driveTrain.stopDriveTrain();
                stop();
            }

            telemetry.update();
        }
    }
}
