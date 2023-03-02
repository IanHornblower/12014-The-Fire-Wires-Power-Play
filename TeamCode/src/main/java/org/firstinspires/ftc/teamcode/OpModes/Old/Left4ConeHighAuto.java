package org.firstinspires.ftc.teamcode.OpModes.Old;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import static org.firstinspires.ftc.teamcode.SuperDuperUsefulStuff.OpModeStuff.OpModeInformations.*;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ActionSystem.ActionSequence;
import org.firstinspires.ftc.teamcode.ActionSystem.ActionSequenceRunner;
import org.firstinspires.ftc.teamcode.Auto.HighTrajectories;
import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.SuperDuperUsefulStuff.OpModeStuff.OpModeInformations;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.ConeManipulator;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Intake;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Lift;
import org.firstinspires.ftc.teamcode.hardware.subsystems.RearCamera;
import org.firstinspires.ftc.teamcode.util.Color;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.Timer;
import org.firstinspires.ftc.teamcode.vision.CombinedTracker;

@Disabled
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

        HighTrajectories trajectories = new HighTrajectories(rob);


        // Go to high Pole (DT)
        driveTrain.releasetOdom();
        //driveTrain.followTrajectory(trajectories.toHighPoleFromStart);
        //driveTrain.waitFor(()-> rob.lift.getPosition() == Lift.cone5); // Adjust where it starts moving after drop
        driveTrain.waitSeconds(0);
        // Go to cone stack
      //  driveTrain.followTrajectory(trajectories.toConeStackFromHighPole);
       // driveTrain.waitSeconds(5);
    //    driveTrain.followTrajectory(trajectories.toHighPoleFromConeStack);

        // Go to high pole (LIFT)
        lift.closeClaw();
        //lift.intakeOut();
        lift.fourbarTo(550, true);
        lift.liftTo(Lift.LIFT.SUPERHIGH, true);

        // Drop Cone and return to intaking position
        lift.waitFor(()-> driveTrainRunner.isActionComplete(1));

        lift.dropAndReturn(Lift.cone5);
        lift.waitFor(()-> driveTrainRunner.isActionComplete(3));
        lift.closeClaw();
        lift.waitSeconds(0.28);
        lift.liftTo(Lift.LIFT.SUPERHIGH, true);



        rob.release();
        rob.intake.setDirection(Intake.DIRECTION.back);
        rob.coneManipulator.close();
        sleep(500);
        rob.coneManipulator.setPosition(900); // Tune up value to be inside the 18" cube


        RearCamera.State location = RearCamera.State.NONE;

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
            case MIDDLE:

                break;
            case RIGHT:

                break;
        }


        waitForStart();

        rob.rearCamera.camera.stopStreaming();
        rob.rearCamera.camera.closeCameraDevice();

        timer.reset();

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
               // stop();
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
            telemetry.addLine();
            telemetry.addData("Lift Step", liftRunner.getCurrentAction());
            telemetry.addData("Drive Train Step", driveTrainRunner.getCurrentAction());
            telemetry.update();
        }
    }
}
