package org.firstinspires.ftc.teamcode.OpModes.Actual;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Auto.HighTrajectories;
import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.ConeManipulator;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Intake;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Lift;
import org.firstinspires.ftc.teamcode.hardware.subsystems.RearCamera;
import org.firstinspires.ftc.teamcode.util.Color;
import org.firstinspires.ftc.teamcode.util.MathUtil;
import org.firstinspires.ftc.teamcode.util.Timer;
import org.firstinspires.ftc.teamcode.vision.CombinedTracker;

@Autonomous(name="Right Side 1+5 on High", group = "AUTO")
public class RightHighAuto extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        // Setup Robot Hardware & Define OpMode Type
        Robot rob = new Robot(hardwareMap, telemetry, Robot.OPMODE_TYPE.AUTO);

        // Fancify Telemetry
        telemetry = rob.getTelemetry();
        telemetry.setMsTransmissionInterval(80);

        // Start Pose
        Pose2d start = new Pose2d(-39, 62, Math.toRadians(90));
        rob.driveTrain.setPoseEstimate(start);

        // Reset 4bar enc
        rob.coneManipulator.resetEnc();

        // Init Robot
        rob.init();

        // Timer for init & match Timer
        Timer timer = new Timer();
        timer.start();

        if(isStopRequested()) return;

        Lift.Kg = 0.12;
        Lift.downSpeed = -0.26;
        ConeManipulator.kCos = 0.008;

        HighTrajectories trajectories = new HighTrajectories(rob);

        TrajectorySequence left = rob.driveTrain.trajectorySequenceBuilder(start)
                .setReversed(true)


                .addTemporalMarker(3.3,() -> {
                    rob.coneManipulator.setPosition(250);
                    rob.coneManipulator.open();
                })
                .addTemporalMarker(3.5, ()-> {
                    rob.coneManipulator.setPosition(1920);
                    rob.lift.setPosition(0);
                })



                .addTemporalMarker(5.55, ()-> {
                    rob.coneManipulator.setPosition(2380);
                })

                .addTemporalMarker(5.65, ()-> {
                    rob.coneManipulator.close();
                })


                .addTemporalMarker(6.2, ()-> {
                 rob.lift.setPosition(Lift.highPoleBroken); //-75
                 rob.coneManipulator.setPosition(850);
                })

                .addTemporalMarker(7.9, ()-> {
                    rob.coneManipulator.setPosition(250);

                })
                .addTemporalMarker(8, ()-> {
                    rob.coneManipulator.open();
                })


                .addTemporalMarker(8.5, ()-> {
                    rob.coneManipulator.setPosition(2000);
                    rob.lift.setPosition(0);
                })





                // Cycle 2

                .addTemporalMarker(9.55, ()-> {
                    rob.coneManipulator.setPosition(2550);
                })

                .addTemporalMarker(9.65, ()-> {
                    rob.coneManipulator.close();
                })

                .addTemporalMarker(10.1, ()-> {
                    rob.lift.setPosition(Lift.highPoleBroken-75);
                    rob.coneManipulator.setPosition(850);
                })



                .addTemporalMarker(11.8, ()-> {
                    rob.coneManipulator.setPosition(250);

                })
                .addTemporalMarker(11.9, ()-> {
                    rob.coneManipulator.open();
                })

                .addTemporalMarker(12, ()-> {
                    rob.coneManipulator.setPosition(2050);
                    rob.lift.setPosition(0);
                })



                // Cycle 3

                .addTemporalMarker(14.05-0.6, ()-> {
                    rob.coneManipulator.setPosition(2700);
                })

                .addTemporalMarker(14.15-0.6, ()-> {
                    rob.coneManipulator.close();
                })



                .addTemporalMarker(14.1, ()-> {
                    rob.lift.setPosition(Lift.highPoleBroken-75);
                    rob.coneManipulator.setPosition(850);
                })

                .addTemporalMarker(15.8, ()-> {
                    rob.coneManipulator.setPosition(400);
                    rob.coneManipulator.open();
                })
                .addTemporalMarker(16.15, ()-> {
                    rob.coneManipulator.setPosition(2100);
                    rob.lift.setPosition(0);
                  rob.intake.setPower(1);
                })




                // Cycle 4

                .addTemporalMarker(17.55, ()-> {
                    rob.coneManipulator.setPosition(2700);
                })

                .addTemporalMarker(17.65, ()-> {
                    rob.coneManipulator.close();
                })



                .addTemporalMarker(18, ()-> {
                    rob.lift.setPosition(Lift.highPoleBroken-75);
                    rob.coneManipulator.setPosition(850);
                   rob.intake.setPower(0);
                })

                .addTemporalMarker(20.1, ()-> {
                    rob.coneManipulator.setPosition(400);
                })
                .addTemporalMarker(20.2, ()-> {
                    rob.coneManipulator.open();
                })
                .addTemporalMarker(20.6, ()-> {
                    rob.coneManipulator.setPosition(2150);
                    rob.lift.setPosition(0);
                    rob.intake.setPower(0);
                })
                .addTemporalMarker(21.2, ()-> {
                    rob.coneManipulator.setPosition(0);
                })


                .setConstraints(SampleMecanumDrive.getVelocityConstraint(80, Math.toRadians(300), 14.2), SampleMecanumDrive.getAccelerationConstraint(80))

                // To Main Pole
                .addTemporalMarker(()-> {
                    rob.intake.setPower(1);
                    rob.coneManipulator.setPosition(850);
                   // rob.coneManipulator.setPosition(250);
                })
                .splineTo(new Vector2d(-34, 38), Math.toRadians(270))
                .splineTo(new Vector2d(-40, 6), Math.toRadians(270))

                .setReversed(false)
                .addTemporalMarker(()-> rob.lift.setPosition(Lift.highPoleBroken-75))
                .splineTo(new Vector2d(-38, 18), Math.toRadians(90))
                .setReversed(true)

                .addTemporalMarker(()-> {
                    //rob.coneManipulator.setPosition(900);
                })

                .setConstraints(SampleMecanumDrive.getVelocityConstraint(55, Math.toRadians(180), 14.2), SampleMecanumDrive.getAccelerationConstraint(55))

                .splineTo(new Vector2d(-31.5, 7.5), Math.toRadians(-55))
                .back(0.8)
                .addTemporalMarker(()-> rob.intake.setPower(0))

                .resetConstraints()



                // Cycle 1

                // To Cone Stack
                .setReversed(false)
                .splineTo(new Vector2d(-34, 11), Math.toRadians(180))
                .splineTo(new Vector2d(-61.8, 13), Math.toRadians(178))
                .waitSeconds(0.5)

                .setReversed(true)
                .splineTo(new Vector2d(-52, 12), Math.toRadians(0))
                .splineTo(HighTrajectories.cycle1r.vec(), HighTrajectories.cycle1r.getHeading()) // -31, -4
                .back(0.5)






                // Cycle 2

                // To Cone Stack
                .setReversed(false)
                .splineTo(new Vector2d(-50, 11.5), Math.toRadians(180))
                .splineTo(new Vector2d(-62.1, 11.3), Math.toRadians(178))
                .waitSeconds(0.4)



                .setReversed(true)
                .splineTo(new Vector2d(-52, 12), Math.toRadians(0))
                .splineTo(HighTrajectories.cycle2r.vec(), HighTrajectories.cycle2r.getHeading()) // -31, -4
                .back(0.5)

                // Cycle 3

                // To Cone Stack
                .waitSeconds(0.3)
                .setReversed(false)
                .splineTo(new Vector2d(-50, 10.2), Math.toRadians(180))
                .splineTo(new Vector2d(-62.2, 10), Math.toRadians(175))
                .waitSeconds(0.5)



                .setReversed(true)
                .splineTo(new Vector2d(-50, 12), Math.toRadians(0))
                .splineTo(HighTrajectories.cycle3r.vec(), HighTrajectories.cycle3r.getHeading()) // -31, -4
                .back(0.5)




                // Cycle 4

                // To Cone Stack
                .setReversed(false)
                .splineTo(new Vector2d(-50, 9), Math.toRadians(180))
                .splineTo(new Vector2d(-63, 8.8), Math.toRadians(180))
                .waitSeconds(0.5)

                .setReversed(true)
                .splineTo(new Vector2d(-50, 11), Math.toRadians(0))
                .splineTo(HighTrajectories.cycle4r.vec(), HighTrajectories.cycle4r.getHeading()) // -31, -4
                .back(1.5)


                .waitSeconds(1)
                .setReversed(false)

                // Middle
                .splineTo(new Vector2d(-33 , 22), Math.toRadians(90))

                //Left

                //.splineTo(new Vector2d(-62, 9), Math.toRadians(180))

                // Right


                //.splineTo(new Vector2d(-24, 6), Math.toRadians(0)) // Avoid High and Mid Pole
                //.forward(4) // Don't hit poles while turning
                //.splineTo(new Vector2d(-16, 22), Math.toRadians(90)) // End Pos

                //.addTemporalMarker(()-> rob.driveTrain.followTrajectorySequenceAsync())
                .build();




        rob.release();
        rob.intake.setDirection(Intake.DIRECTION.back);
        rob.coneManipulator.close();
        sleep(500);
        rob.coneManipulator.setPosition(900);


        RearCamera.State location = RearCamera.State.NONE;


        while(opModeInInit() && !isStopRequested()) {
            telemetry.addLine(Color.WHITE.format("INIT FINISHED")); // Do this later | fancy title
            //telemetry.addLine(rob.rearCamera.getTelemetry());
           // telemetry.addData("pos", rob.rearCamera.getSleeveLocation().toString());
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
               // rob.driveTrain.followTrajectorySequenceAsync(middle);
                break;
            case RIGHT:
               // rob.driveTrain.followTrajectorySequenceAsync(right);
                break;
            default:
              //  rob.driveTrain.followTrajectorySequenceAsync(middle);
                break;
        }

        //rob.driveTrain.followTrajectorySequenceAsync(left);
        rob.driveTrain.followTrajectorySequenceAsync(left);


        waitForStart();

        timer.reset();

        //rob.rearCamera.camera.stopStreaming();
        //rob.rearCamera.camera.closeCameraDevice();

        while(opModeIsActive() && !isStopRequested()) {
            rob.driveTrain.update();
            rob.update();


            if(!rob.driveTrain.isBusy()) {
                rob.driveTrain.setMotorPowers(0, 0,0, 0);
                //stop();
                timer.pause();
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
            telemetry.addLine(Color.YELLOW.format(rob.mogus.get38pxTwerk()));
            telemetry.update();
        }
    }
}
