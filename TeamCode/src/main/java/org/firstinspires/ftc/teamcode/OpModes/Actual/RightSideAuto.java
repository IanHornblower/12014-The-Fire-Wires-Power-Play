package org.firstinspires.ftc.teamcode.OpModes.Actual;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.ActionSystem.ActionSequence;
import org.firstinspires.ftc.teamcode.ActionSystem.ActionSequenceRunner;
import org.firstinspires.ftc.teamcode.ActionSystem.actions.LiftSetPosition;
import org.firstinspires.ftc.teamcode.ActionSystem.actions.Wait;
import org.firstinspires.ftc.teamcode.Roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.Roadrunner.trajectorysequence.TrajectorySequenceBuilder;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.ConeManipulator;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Intake;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Lift;
import org.firstinspires.ftc.teamcode.hardware.subsystems.RearCamera;
import org.firstinspires.ftc.teamcode.util.Color;
import org.firstinspires.ftc.teamcode.util.Timer;
import org.firstinspires.ftc.teamcode.vision.CombinedTracker;

@Autonomous
public class RightSideAuto extends LinearOpMode {
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

        ActionSequence lift = new ActionSequence();
        ActionSequence driveTrain = new ActionSequence(rob);

        ActionSequenceRunner driveTrainRunner = new ActionSequenceRunner(rob);
        driveTrainRunner.setActionSequence(driveTrain);

        ActionSequenceRunner liftRunner = new ActionSequenceRunner(rob);
        liftRunner.setActionSequence(lift);

        if(isStopRequested()) return;

        TrajectorySequence left = rob.driveTrain.trajectorySequenceBuilder(start)
                .strafeTo(new Vector2d(-10, -58))
                .setReversed(true)
                .splineTo(new Vector2d(-10, -46), Math.toRadians(90))
                .splineTo(new Vector2d(-10, -11), Math.toRadians(90))

                .addTemporalMarker(()->  {
                    rob.lift.setPosition(Lift.highPole+100);
                    rob.intake.setDirection(Intake.DIRECTION.front);
                    rob.coneManipulator.setPosition(ConeManipulator.V4BPreset.DROP);
                })

                .setReversed(false)
                .splineTo(new Vector2d(0, -20), Math.toRadians(-45))
                .waitSeconds(0.5) // Wait at First Pole
                .addTemporalMarker(()-> {
                    rob.coneManipulator.setPosition(ConeManipulator.V4BPreset.CORRECT_RIGHT);
                })
                .waitSeconds(0.2)
                .addTemporalMarker(()-> {
                    rob.coneManipulator.open();
                })
                .waitSeconds(0.2)
                .addTemporalMarker(()-> {
                    rob.coneManipulator.close();
                    rob.coneManipulator.setPosition(ConeManipulator.V4BPreset.VERTICAL);
                    rob.lift.setPosition(0);
                })

                // Cycle 1

                .setReversed(true)
                .splineTo(new Vector2d(-18, -10), Math.toRadians(180))

                .addTemporalMarker(()-> {
                    rob.intake.setDirection(Intake.DIRECTION.front);
                    rob.coneManipulator.setPosition(ConeManipulator.V4BPreset.STACK1);
                    rob.coneManipulator.open();
                })

                .splineTo(new Vector2d(-56.5, -5.5), Math.toRadians(175))

                .addTemporalMarker(()->  rob.coneManipulator.close())
                .waitSeconds(0.5)
                .addTemporalMarker(()-> {
                    rob.lift.setPosition(Lift.highPole+50);
                    rob.coneManipulator.setPosition(ConeManipulator.V4BPreset.DROP);
                })
                .setReversed(false)
                .splineTo(new Vector2d(-39, -8), Math.toRadians(0))
                .splineTo(new Vector2d(-22, 0), Math.toRadians(45))

                .waitSeconds(0.1) // Wait at First Pole
                .addTemporalMarker(()-> {
                    rob.coneManipulator.setPosition(ConeManipulator.V4BPreset.CORRECT_RIGHT);
                })
                .waitSeconds(0.2)
                .addTemporalMarker(()-> {
                    rob.coneManipulator.open();
                })
                .waitSeconds(0.2)
                .addTemporalMarker(()-> {
                    rob.coneManipulator.close();
                    rob.coneManipulator.setPosition(ConeManipulator.V4BPreset.VERTICAL);
                    rob.lift.setPosition(0);
                })

                // Cycle 2

                .setReversed(true)
                .splineTo(new Vector2d(-39, -8), Math.toRadians(180))

                .addTemporalMarker(()-> {
                    rob.intake.setDirection(Intake.DIRECTION.front);
                    rob.coneManipulator.setPosition(ConeManipulator.V4BPreset.STACK2);
                    rob.coneManipulator.open();
                })

                .splineTo(new Vector2d(-55, -6), Math.toRadians(178))

                .addTemporalMarker(()->  rob.coneManipulator.close())
                .waitSeconds(0.5)
                .addTemporalMarker(()-> {
                    rob.lift.setPosition(Lift.highPole+50);
                    rob.coneManipulator.setPosition(ConeManipulator.V4BPreset.DROP);
                })
                .setReversed(false)
                .splineTo(new Vector2d(-39, -8), Math.toRadians(0))
                .splineTo(new Vector2d(-22, 0), Math.toRadians(45))

                .waitSeconds(0.1) // Wait at First Pole
                .addTemporalMarker(()-> {
                    rob.coneManipulator.setPosition(ConeManipulator.V4BPreset.CORRECT_RIGHT);
                })
                .waitSeconds(0.2)
                .addTemporalMarker(()-> {
                    rob.coneManipulator.open();
                })
                .waitSeconds(0.2)
                .addTemporalMarker(()-> {
                    rob.coneManipulator.close();
                    rob.coneManipulator.setPosition(ConeManipulator.V4BPreset.VERTICAL);
                    rob.lift.setPosition(0);
                })

               // .waitSeconds(999)

                // Cycle 3


                .setReversed(true)
                .splineTo(new Vector2d(-39, -8), Math.toRadians(180))

                .addTemporalMarker(()-> {
                    rob.intake.setDirection(Intake.DIRECTION.front);
                    rob.coneManipulator.setPosition(ConeManipulator.V4BPreset.STACK3);
                    rob.coneManipulator.open();
                })

                .splineTo(new Vector2d(-55, -6), Math.toRadians(178))

                .addTemporalMarker(()->  rob.coneManipulator.close())
                .waitSeconds(0.5)
                .addTemporalMarker(()-> {
                    rob.lift.setPosition(Lift.highPole+50);
                    rob.coneManipulator.setPosition(ConeManipulator.V4BPreset.DROP);
                })
                .setReversed(false)
                .splineTo(new Vector2d(-39, -8), Math.toRadians(0))
                .splineTo(new Vector2d(-22, 0), Math.toRadians(45))

                .waitSeconds(0.1) // Wait at First Pole
                .addTemporalMarker(()-> {
                    rob.coneManipulator.setPosition(ConeManipulator.V4BPreset.CORRECT_RIGHT);
                })
                .waitSeconds(0.2)
                .addTemporalMarker(()-> {
                    rob.coneManipulator.open();
                })
                .waitSeconds(0.2)
                .addTemporalMarker(()-> {
                    rob.coneManipulator.close();
                    rob.coneManipulator.setPosition(ConeManipulator.V4BPreset.VERTICAL);
                    rob.lift.setPosition(0);
                })

                // Park

                .setReversed(true)
                .splineTo(new Vector2d(-39, -6), Math.toRadians(180)) // Left
                .splineTo(new Vector2d(-54, -6), Math.toRadians(180))
                .addTemporalMarker(()-> rob.coneManipulator.setPosition(25))
                .build();

        TrajectorySequence middle = rob.driveTrain.trajectorySequenceBuilder(start)
                .strafeTo(new Vector2d(-10, -58))
                .setReversed(true)
                .splineTo(new Vector2d(-10, -46), Math.toRadians(90))
                .splineTo(new Vector2d(-10, -11), Math.toRadians(90))

                .addTemporalMarker(()->  {
                    rob.lift.setPosition(Lift.highPole+100);
                    rob.intake.setDirection(Intake.DIRECTION.front);
                    rob.coneManipulator.setPosition(ConeManipulator.V4BPreset.DROP);
                })

                .setReversed(false)
                .splineTo(new Vector2d(0, -20), Math.toRadians(-45))
                .waitSeconds(0.5) // Wait at First Pole
                .addTemporalMarker(()-> {
                    rob.coneManipulator.setPosition(ConeManipulator.V4BPreset.CORRECT_RIGHT);
                })
                .waitSeconds(0.2)
                .addTemporalMarker(()-> {
                    rob.coneManipulator.open();
                })
                .waitSeconds(0.2)
                .addTemporalMarker(()-> {
                    rob.coneManipulator.close();
                    rob.coneManipulator.setPosition(ConeManipulator.V4BPreset.VERTICAL);
                    rob.lift.setPosition(0);
                })

                // Cycle 1

                .setReversed(true)
                .splineTo(new Vector2d(-18, -10), Math.toRadians(180))

                .addTemporalMarker(()-> {
                    rob.intake.setDirection(Intake.DIRECTION.front);
                    rob.coneManipulator.setPosition(ConeManipulator.V4BPreset.STACK1);
                    rob.coneManipulator.open();
                })

                .splineTo(new Vector2d(-56.5, -5.5), Math.toRadians(175))

                .addTemporalMarker(()->  rob.coneManipulator.close())
                .waitSeconds(0.5)
                .addTemporalMarker(()-> {
                    rob.lift.setPosition(Lift.highPole+50);
                    rob.coneManipulator.setPosition(ConeManipulator.V4BPreset.DROP);
                })
                .setReversed(false)
                .splineTo(new Vector2d(-39, -8), Math.toRadians(0))
                .splineTo(new Vector2d(-22, 0), Math.toRadians(45))

                .waitSeconds(0.1) // Wait at First Pole
                .addTemporalMarker(()-> {
                    rob.coneManipulator.setPosition(ConeManipulator.V4BPreset.CORRECT_RIGHT);
                })
                .waitSeconds(0.2)
                .addTemporalMarker(()-> {
                    rob.coneManipulator.open();
                })
                .waitSeconds(0.2)
                .addTemporalMarker(()-> {
                    rob.coneManipulator.close();
                    rob.coneManipulator.setPosition(ConeManipulator.V4BPreset.VERTICAL);
                    rob.lift.setPosition(0);
                })

                // Cycle 2

                .setReversed(true)
                .splineTo(new Vector2d(-39, -8), Math.toRadians(180))

                .addTemporalMarker(()-> {
                    rob.intake.setDirection(Intake.DIRECTION.front);
                    rob.coneManipulator.setPosition(ConeManipulator.V4BPreset.STACK2);
                    rob.coneManipulator.open();
                })

                .splineTo(new Vector2d(-55, -6), Math.toRadians(178))

                .addTemporalMarker(()->  rob.coneManipulator.close())
                .waitSeconds(0.5)
                .addTemporalMarker(()-> {
                    rob.lift.setPosition(Lift.highPole+50);
                    rob.coneManipulator.setPosition(ConeManipulator.V4BPreset.DROP);
                })
                .setReversed(false)
                .splineTo(new Vector2d(-39, -8), Math.toRadians(0))
                .splineTo(new Vector2d(-22, 0), Math.toRadians(45))

                .waitSeconds(0.1) // Wait at First Pole
                .addTemporalMarker(()-> {
                    rob.coneManipulator.setPosition(ConeManipulator.V4BPreset.CORRECT_RIGHT);
                })
                .waitSeconds(0.2)
                .addTemporalMarker(()-> {
                    rob.coneManipulator.open();
                })
                .waitSeconds(0.2)
                .addTemporalMarker(()-> {
                    rob.coneManipulator.close();
                    rob.coneManipulator.setPosition(ConeManipulator.V4BPreset.VERTICAL);
                    rob.lift.setPosition(0);
                })

                // .waitSeconds(999)

                // Cycle 3


                .setReversed(true)
                .splineTo(new Vector2d(-39, -8), Math.toRadians(180))

                .addTemporalMarker(()-> {
                    rob.intake.setDirection(Intake.DIRECTION.front);
                    rob.coneManipulator.setPosition(ConeManipulator.V4BPreset.STACK3);
                    rob.coneManipulator.open();
                })

                .splineTo(new Vector2d(-55, -6), Math.toRadians(178))

                .addTemporalMarker(()->  rob.coneManipulator.close())
                .waitSeconds(0.5)
                .addTemporalMarker(()-> {
                    rob.lift.setPosition(Lift.highPole+50);
                    rob.coneManipulator.setPosition(ConeManipulator.V4BPreset.DROP);
                })
                .setReversed(false)
                .splineTo(new Vector2d(-39, -8), Math.toRadians(0))
                .splineTo(new Vector2d(-22, 0), Math.toRadians(45))

                .waitSeconds(0.1) // Wait at First Pole
                .addTemporalMarker(()-> {
                    rob.coneManipulator.setPosition(ConeManipulator.V4BPreset.CORRECT_RIGHT);
                })
                .waitSeconds(0.2)
                .addTemporalMarker(()-> {
                    rob.coneManipulator.open();
                })
                .waitSeconds(0.2)
                .addTemporalMarker(()-> {
                    rob.coneManipulator.close();
                    rob.coneManipulator.setPosition(ConeManipulator.V4BPreset.VERTICAL);
                    rob.lift.setPosition(0);
                })

                // Park

                .setReversed(true)
                .splineTo(new Vector2d(-32, -6), Math.toRadians(175)) // Middle
                .addTemporalMarker(()-> rob.coneManipulator.setPosition(25))
                .build();

        TrajectorySequence right = rob.driveTrain.trajectorySequenceBuilder(start)
                .strafeTo(new Vector2d(-10, -58))
                .setReversed(true)
                .splineTo(new Vector2d(-10, -46), Math.toRadians(90))
                .splineTo(new Vector2d(-10, -11), Math.toRadians(90))

                .addTemporalMarker(()->  {
                    rob.lift.setPosition(Lift.highPole+100);
                    rob.intake.setDirection(Intake.DIRECTION.front);
                    rob.coneManipulator.setPosition(ConeManipulator.V4BPreset.DROP);
                })

                .setReversed(false)
                .splineTo(new Vector2d(0, -20), Math.toRadians(-45))
                .waitSeconds(0.5) // Wait at First Pole
                .addTemporalMarker(()-> {
                    rob.coneManipulator.setPosition(ConeManipulator.V4BPreset.CORRECT_RIGHT);
                })
                .waitSeconds(0.2)
                .addTemporalMarker(()-> {
                    rob.coneManipulator.open();
                })
                .waitSeconds(0.2)
                .addTemporalMarker(()-> {
                    rob.coneManipulator.close();
                    rob.coneManipulator.setPosition(ConeManipulator.V4BPreset.VERTICAL);
                    rob.lift.setPosition(0);
                })

                // Cycle 1

                .setReversed(true)
                .splineTo(new Vector2d(-18, -10), Math.toRadians(180))

                .addTemporalMarker(()-> {
                    rob.intake.setDirection(Intake.DIRECTION.front);
                    rob.coneManipulator.setPosition(ConeManipulator.V4BPreset.STACK1);
                    rob.coneManipulator.open();
                })

                .splineTo(new Vector2d(-56.5, -5.5), Math.toRadians(175))

                .addTemporalMarker(()->  rob.coneManipulator.close())
                .waitSeconds(0.5)
                .addTemporalMarker(()-> {
                    rob.lift.setPosition(Lift.highPole+50);
                    rob.coneManipulator.setPosition(ConeManipulator.V4BPreset.DROP);
                })
                .setReversed(false)
                .splineTo(new Vector2d(-39, -8), Math.toRadians(0))
                .splineTo(new Vector2d(-22, 0), Math.toRadians(45))

                .waitSeconds(0.1) // Wait at First Pole
                .addTemporalMarker(()-> {
                    rob.coneManipulator.setPosition(ConeManipulator.V4BPreset.CORRECT_RIGHT);
                })
                .waitSeconds(0.2)
                .addTemporalMarker(()-> {
                    rob.coneManipulator.open();
                })
                .waitSeconds(0.2)
                .addTemporalMarker(()-> {
                    rob.coneManipulator.close();
                    rob.coneManipulator.setPosition(ConeManipulator.V4BPreset.VERTICAL);
                    rob.lift.setPosition(0);
                })

                // Cycle 2

                .setReversed(true)
                .splineTo(new Vector2d(-39, -8), Math.toRadians(180))

                .addTemporalMarker(()-> {
                    rob.intake.setDirection(Intake.DIRECTION.front);
                    rob.coneManipulator.setPosition(ConeManipulator.V4BPreset.STACK2);
                    rob.coneManipulator.open();
                })

                .splineTo(new Vector2d(-55, -6), Math.toRadians(178))

                .addTemporalMarker(()->  rob.coneManipulator.close())
                .waitSeconds(0.5)
                .addTemporalMarker(()-> {
                    rob.lift.setPosition(Lift.highPole+50);
                    rob.coneManipulator.setPosition(ConeManipulator.V4BPreset.DROP);
                })
                .setReversed(false)
                .splineTo(new Vector2d(-39, -8), Math.toRadians(0))
                .splineTo(new Vector2d(-22, 0), Math.toRadians(45))

                .waitSeconds(0.1) // Wait at First Pole
                .addTemporalMarker(()-> {
                    rob.coneManipulator.setPosition(ConeManipulator.V4BPreset.CORRECT_RIGHT);
                })
                .waitSeconds(0.2)
                .addTemporalMarker(()-> {
                    rob.coneManipulator.open();
                })
                .waitSeconds(0.2)
                .addTemporalMarker(()-> {
                    rob.coneManipulator.close();
                    rob.coneManipulator.setPosition(ConeManipulator.V4BPreset.VERTICAL);
                    rob.lift.setPosition(0);
                })

                // .waitSeconds(999)

                // Cycle 3


                .setReversed(true)
                .splineTo(new Vector2d(-39, -8), Math.toRadians(180))

                .addTemporalMarker(()-> {
                    rob.intake.setDirection(Intake.DIRECTION.front);
                    rob.coneManipulator.setPosition(ConeManipulator.V4BPreset.STACK3);
                    rob.coneManipulator.open();
                })

                .splineTo(new Vector2d(-55, -6), Math.toRadians(178))

                .addTemporalMarker(()->  rob.coneManipulator.close())
                .waitSeconds(0.5)
                .addTemporalMarker(()-> {
                    rob.lift.setPosition(Lift.highPole+50);
                    rob.coneManipulator.setPosition(ConeManipulator.V4BPreset.DROP);
                })
                .setReversed(false)
                .splineTo(new Vector2d(-39, -8), Math.toRadians(0))
                .splineTo(new Vector2d(-22, 0), Math.toRadians(45))

                .waitSeconds(0.1) // Wait at First Pole
                .addTemporalMarker(()-> {
                    rob.coneManipulator.setPosition(ConeManipulator.V4BPreset.CORRECT_RIGHT);
                })
                .waitSeconds(0.2)
                .addTemporalMarker(()-> {
                    rob.coneManipulator.open();
                })
                .waitSeconds(0.2)
                .addTemporalMarker(()-> {
                    rob.coneManipulator.close();
                    rob.coneManipulator.setPosition(ConeManipulator.V4BPreset.VERTICAL);
                    rob.lift.setPosition(0);
                })


                // Cycle 4

                // Park

                .setReversed(true)

                .splineTo(new Vector2d(-27, -7.5), Math.toRadians(175)) // Right
                .forward(24)
                .addTemporalMarker(()-> rob.coneManipulator.setPosition(25))
                .build();



        rob.intake.setDirection(Intake.DIRECTION.back);
        rob.coneManipulator.close();
        sleep(500);
        rob.coneManipulator.setPosition(ConeManipulator.V4BPreset.DROP);


        CombinedTracker.ParkingPosition location = CombinedTracker.ParkingPosition.CENTER;


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
            case CENTER:
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
            /*
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

             */

            rob.driveTrain.update();
            rob.update();


            if(!rob.driveTrain.isBusy()) {
                rob.driveTrain.setMotorPowers(0, 0,0, 0);
                stop();
            }

            telemetry.addData("step", driveTrainRunner.getCurrentAction());
            telemetry.update();
        }
    }
}
