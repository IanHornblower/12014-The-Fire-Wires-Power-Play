package org.firstinspires.ftc.teamcode.OpModes.Testing;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ActionSystem.ActionSequence;
import org.firstinspires.ftc.teamcode.ActionSystem.ActionSequenceRunner;
import org.firstinspires.ftc.teamcode.ActionSystem.actions.ConeTurn;
import org.firstinspires.ftc.teamcode.ActionSystem.actions.FollowBlueLineToDistance;
import org.firstinspires.ftc.teamcode.ActionSystem.actions.ReturnToDistance;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.util.Color;
import org.firstinspires.ftc.teamcode.util.Timer;

@Autonomous
public class AutoTesting extends LinearOpMode {

    public static double kP = 0.001;

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
        ActionSequence claw = new ActionSequence();

        ActionSequence driveTrain = new ActionSequence();

        ActionSequenceRunner driveTrainRunner = new ActionSequenceRunner(rob);
        driveTrainRunner.setActionSequence(driveTrain);

        ActionSequenceRunner liftRunner = new ActionSequenceRunner(rob);
        liftRunner.setActionSequence(lift);

        ActionSequenceRunner clawRunner = new ActionSequenceRunner(rob);
        clawRunner.setActionSequence(claw);

        while(opModeInInit() && !isStopRequested()) {
            telemetry.addLine(Color.WHITE.format("INIT FINISHED")); // Do this later | fancy title
            telemetry.addData("Time Since Init", Color.LIME.format(timer.currentSeconds()));
            telemetry.update();
        }

        //driveTrain.addAction(new EncoderDrive(rob, Math.toRadians(45), 0.5, Math.toRadians(0), 1000, true, true));
        //driveTrain.addAction(new BasicTurn(rob, Math.toRadians(90)));]

        driveTrain.addAction(new ConeTurn(rob));

        waitForStart();

        while(opModeIsActive() && !isStopRequested()) {




            //telemetry.addData("Cone Error", rob.poleDetector.getCameraError());
            telemetry.update();

            if(!driveTrainRunner.isComplete()) {
                try {
                    driveTrainRunner.update();
                } catch (Exception e) {
                    e.printStackTrace();
                }
            }

             /**
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
                rob.t265.stop();
                rob.driveTrain.stopDriveTrain();
                stop();
            }

             */
        }
    }
}
