package org.firstinspires.ftc.teamcode.OpModes.Actual;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import static org.firstinspires.ftc.teamcode.SuperDuperUsefulStuff.OpModeStuff.OpModeInformations.*;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ActionSystem.ActionSequence;
import org.firstinspires.ftc.teamcode.ActionSystem.ActionSequenceRunner;
import org.firstinspires.ftc.teamcode.SuperDuperUsefulStuff.OpModeStuff.OpModeInformations;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.ConeManipulator;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Intake;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Lift;
import org.firstinspires.ftc.teamcode.util.Color;
import org.firstinspires.ftc.teamcode.util.Timer;
import org.firstinspires.ftc.teamcode.vision.CombinedTracker;

@Autonomous(name = "Left Side 1+3 on High", group = "AUTO")
public class Left4ConeHighAuto extends LinearOpMode  {
    @Override
    public void runOpMode() throws InterruptedException {
        // Setup Robot Hardware & Define OpMode Type
        Robot rob = new Robot(hardwareMap, telemetry, Robot.OPMODE_TYPE.AUTO);

        // Fancify Telemetry
        telemetry = rob.getTelemetry();

        // Start Pose
        Pose2d start = new Pose2d(-39, -62, Math.toRadians(-90));
        rob.driveTrain.setPoseEstimate(start);

        // Init Robot
        rob.init();

        // Timer for init & match Timer
        Timer timer = new Timer();
        timer.start();

        ActionSequence lift = new ActionSequence(rob);
        ActionSequence driveTrain = new ActionSequence(rob);

        ActionSequenceRunner driveTrainRunner = new ActionSequenceRunner(rob);
        driveTrainRunner.setActionSequence(driveTrain);

        ActionSequenceRunner liftRunner = new ActionSequenceRunner(rob);
        liftRunner.setActionSequence(lift);

        if(isStopRequested()) return;

        driveTrain.releasetOdom();
        driveTrain.followTrajectory(rob.driveTrain.trajectoryBuilder(start, true) // To High Pole
                .splineTo(new Vector2d(-33, -38), Math.toRadians(90))
                .splineTo(new Vector2d(-28, -4), Math.toRadians(45))
                .build());
        driveTrain.waitFor(()-> rob.lift.getPosition() == Lift.cone5); // Adjust where it starts moving after drop
        driveTrain.followTrajectory(rob.driveTrain.trajectoryBuilder(start, false) // To Cone Stack
                .splineTo(new Vector2d(-62.5, -11.5), Math.toRadians(180))
                .build());

        lift.closeClaw();
        lift.intakeOut();
        lift.liftTo(Lift.LIFT.HIGH, true);
        lift.fourbarTo(ConeManipulator.V4BPreset.DROP);

        // Drop Cone and return to intaking position
        lift.waitFor(()-> driveTrainRunner.isActionComplete(1));
        lift.intakeOff();
        lift.dropAndReturn(Lift.cone5);

        //
        lift.waitFor(()-> driveTrainRunner.isActionComplete(3));
        lift.fourbarTo(ConeManipulator.V4BPreset.DROP);
        lift.closeClaw();
        lift.waitSeconds(0.28);
        lift.liftTo(Lift.LIFT.HIGH, true);
        lift.fourbarTo(ConeManipulator.V4BPreset.VERTICAL);

        lift.waitFor(()-> driveTrainRunner.isActionComplete(1));
        lift.intakeOff();
        lift.dropAndReturn(Lift.cone5);
        


        rob.release();
        rob.intake.setDirection(Intake.DIRECTION.back);
        rob.coneManipulator.close();
        sleep(500);
        rob.coneManipulator.setPosition(ConeManipulator.V4BPreset.DROP);

        CombinedTracker.ParkingPosition location = CombinedTracker.ParkingPosition.CENTER;

        while(opModeInInit() && !isStopRequested()) {
            telemetry.addLine(Color.WHITE.format("INIT FINISHED")); // Do this later | fancy title
            telemetry.addData("Time Since Init", Color.LIME.format(timer.currentSeconds()));
            telemetry.addData("distance reading", rob.intake.back.getDistance(DistanceUnit.MM));
            if(rob.intake.back.getDistance(DistanceUnit.MM) < 100 || rob.intake.front.getDistance(DistanceUnit.MM) < 100) {
                telemetry.addLine(Color.LIME.format("AUTO SETUP CORRECT"));
            }
            else {
                telemetry.addLine(Color.RED.format("DOFUS AUTO IS SET UP WRONG"));
            }
            telemetry.update();

            location = rob.rearCamera.getSleeveLocation();

            rob.update();

            if(isStopRequested()) return;
        }

        if(isStopRequested()) return;

        switch (location) {
            case LEFT:

                break;
            case CENTER:

                break;
            case RIGHT:

                break;
        }


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
                rob.retract();
                stop();
            }

            telemetry.addData("Lift Step", liftRunner.getCurrentAction());
            telemetry.addData("Drive Train Step", driveTrainRunner.getCurrentAction());
            telemetry.update();
        }
    }
}
