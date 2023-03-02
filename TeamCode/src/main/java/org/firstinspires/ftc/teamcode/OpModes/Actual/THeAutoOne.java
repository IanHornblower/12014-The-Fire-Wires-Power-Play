package org.firstinspires.ftc.teamcode.OpModes.Actual;

import com.acmerobotics.dashboard.config.Config;
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

@Config
@Autonomous(name="THIS ONE NOWOWOWOW", group = "AUTO")
public class THeAutoOne extends LinearOpMode {

    public static double timeFromCycle2 = 0.15;
    public static double timeFromCycle3 = -0.2;
    public static double timeFromCycle4 = 0.4;

    double distance = 1.55;

    @Override
    public void runOpMode() throws InterruptedException {
        // Setup Robot Hardware & Define OpMode Type
        Robot rob = new Robot(hardwareMap, telemetry, Robot.OPMODE_TYPE.AUTO);

        // Fancify Telemetry
        telemetry = rob.getTelemetry();
        telemetry.setMsTransmissionInterval(80);

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

        Lift.Kg = 0.12;
        Lift.downSpeed = -0.26;
        ConeManipulator.kCos = 0.008;

        HighTrajectories trajectories = new HighTrajectories(rob);

        TrajectorySequence left = rob.driveTrain.trajectorySequenceBuilder(start)
                .setReversed(true)

                .setConstraints(SampleMecanumDrive.getVelocityConstraint(55, Math.toRadians(300), 14.2), SampleMecanumDrive.getAccelerationConstraint(80))

                // To Main Pole
                .addTemporalMarker(()-> {
                   // rob.intake.setPower(1);
                    rob.coneManipulator.setPosition(850);
                    // rob.coneManipulator.setPosition(250);
                })
                .splineTo(new Vector2d(-34, -38), Math.toRadians(90))
                .splineTo(new Vector2d(-36, -6), Math.toRadians(90))

                .setReversed(false)
                .addTemporalMarker(()-> rob.lift.setPosition(Lift.highPoleBroken-75))
                .splineTo(new Vector2d(-34, -18), Math.toRadians(270))
                .setReversed(true)

                .addTemporalMarker(()-> {
                    // rob.coneManipulator.setPosition(900);
                })

                .setConstraints(SampleMecanumDrive.getVelocityConstraint(55, Math.toRadians(180), 14.2), SampleMecanumDrive.getAccelerationConstraint(55))

                .splineTo(new Vector2d(-28.5, -6.2), Math.toRadians(62))
                .addTemporalMarker(()-> {
                //    rob.intake.setPower(0);
                    rob.coneManipulator.setPosition(250);
                    rob.coneManipulator.open();

                })
                .waitSeconds(0.3)
                .addTemporalMarker(()-> {
                    rob.coneManipulator.setPosition(1920);
                    rob.lift.setPosition(0);
                })

                .resetConstraints()

                // Cycle 1

                // To Cone Stack
                .setReversed(false)
                .splineTo(new Vector2d(-32, -11.5), Math.toRadians(180))
                .splineTo(new Vector2d(-59.5-distance, -12), Math.toRadians(180))
                .waitSeconds(0.5)

                .setReversed(true)
                .splineTo(new Vector2d(-50, -12), Math.toRadians(0))
                .splineTo(HighTrajectories.cycle1.vec(), HighTrajectories.cycle1.getHeading()) // -31, -4
                //.back(1)




                // Cycle 2

                // To Cone Stack
                .setReversed(false)
                .splineTo(new Vector2d(-59-distance, -12), Math.toRadians(180))
                .waitSeconds(0.5)

                .setReversed(true)
                .splineTo(new Vector2d(-50, -12), Math.toRadians(0))
                .splineTo(HighTrajectories.cycle2.vec(), HighTrajectories.cycle2.getHeading()) // -31, -4
                //.back(1.5)

                // Cycle 3

                // To Cone Stack
                .waitSeconds(0.3)
                .setReversed(false)
                .splineTo(new Vector2d(-58.5-distance, -10), Math.toRadians(180))
                .waitSeconds(0.5)

                .setReversed(true)
                .splineTo(new Vector2d(-50, -12), Math.toRadians(0))
                .splineTo(HighTrajectories.cycle3.vec(), HighTrajectories.cycle3.getHeading()) // -31, -4
                //.back(1.5)



                // Cycle 4
/*
                // To Cone Stack
                .setReversed(false)
                .splineTo(new Vector2d(-58.2-distance, -12), Math.toRadians(185))
                .waitSeconds(0.5)

                .setReversed(true)
                .splineTo(new Vector2d(-50, -12), Math.toRadians(0))
                .splineTo(HighTrajectories.cycle4.vec(), HighTrajectories.cycle4.getHeading()) // -31, -4
                //.back(1.7) //2



                // Cycle 5

                // To Cone Stack
                .setReversed(false)
                .splineTo(new Vector2d(-57.1, -12), Math.toRadians(190))
                .waitSeconds(0.5)

                .setReversed(true)
                .splineTo(new Vector2d(-50, -12), Math.toRadians(0))
                .splineTo(HighTrajectories.cycle5.vec(), HighTrajectories.cycle5.getHeading()) // -31, -4
                //.back(2)

                .addTemporalMarker(()-> {
                    rob.coneManipulator.setPosition(0);
                })

 */

                .setReversed(false)

                .setConstraints(SampleMecanumDrive.getVelocityConstraint(65, Math.toRadians(300), 14.2), SampleMecanumDrive.getAccelerationConstraint(80))//

                .waitSeconds(0.5)
                .addTemporalMarker(()-> rob.coneManipulator.setPosition(0))

                // Middle
                //.splineTo(new Vector2d(-30, -22), Math.toRadians(270))

                //Left

                .splineTo(new Vector2d(-58, -12), Math.toRadians(190))
                //.strafeLeft(8)

                // Right
                //.splineTo(new Vector2d(-24, -6), Math.toRadians(0)) // Avoid High and Mid Pole
                //.forward(4) // Don't hit poles while turning
                //.splineTo(new Vector2d(-9.5, -22), Math.toRadians(270)) // End Pos

                //.addTemporalMarker(()-> rob.driveTrain.followTrajectorySequenceAsync())
                .build();

        TrajectorySequence middle = rob.driveTrain.trajectorySequenceBuilder(start)
                .setReversed(true)

                .addTemporalMarker(3.5,() -> {
                    rob.coneManipulator.setPosition(250);
                    rob.coneManipulator.open();
                })
                .addTemporalMarker(3.8, ()-> {
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
                    rob.lift.setPosition(Lift.highPoleBroken-75);
                    rob.coneManipulator.setPosition(850);
                })

                .addTemporalMarker(7.8, ()-> {
                    rob.coneManipulator.setPosition(250);

                })
                .addTemporalMarker(7.9, ()-> {
                    rob.coneManipulator.open();
                })

                .addTemporalMarker(8.4, ()-> {
                    rob.coneManipulator.setPosition(1980);
                    rob.lift.setPosition(0);
                })

                // Cycle 2

                .addTemporalMarker(9.45-timeFromCycle2, ()-> {
                    rob.coneManipulator.setPosition(2350);
                })

                .addTemporalMarker(9.55-timeFromCycle2, ()-> {
                    rob.coneManipulator.close();
                })

                .addTemporalMarker(10.1-timeFromCycle2, ()-> {
                    rob.lift.setPosition(Lift.highPoleBroken-75);
                    rob.coneManipulator.setPosition(850);
                })

                .addTemporalMarker(11.7, ()-> {
                    rob.coneManipulator.setPosition(250);

                })
                .addTemporalMarker(11.8, ()-> {
                    rob.coneManipulator.open();
                })

                .addTemporalMarker(12.1, ()-> {
                    rob.coneManipulator.setPosition(2150);
                    rob.lift.setPosition(0);
                })


                // Cycle 3

                .addTemporalMarker(14.05-0.6-timeFromCycle3, ()-> {
                    rob.coneManipulator.setPosition(2500);
                })

                .addTemporalMarker(14.15-0.6-timeFromCycle3, ()-> {
                    rob.coneManipulator.close();
                })

                .addTemporalMarker(14.7-0.6-timeFromCycle3, ()-> {
                    rob.lift.setPosition(Lift.highPoleBroken-75);
                    rob.coneManipulator.setPosition(850);
                })

                .addTemporalMarker(16.3-0.6, ()-> {
                    rob.coneManipulator.setPosition(250);
                    rob.coneManipulator.open();
                })
                .addTemporalMarker(16.4-0.4, ()-> {
                    rob.coneManipulator.setPosition(2150);
                    rob.lift.setPosition(0);
                    //    rob.intake.setPower(1);
                })

/*

                // Cycle 4

                .addTemporalMarker(17.45 - timeFromCycle4, ()-> {
                    rob.coneManipulator.setPosition(2550);
                })

                .addTemporalMarker(17.55 - timeFromCycle4, ()-> {
                    rob.coneManipulator.close();
                })

                .addTemporalMarker(18.1 - timeFromCycle4, ()-> {
                    rob.lift.setPosition(Lift.highPoleBroken-75);
                    rob.coneManipulator.setPosition(850);
                    // rob.intake.setPower(0);
                })

                .addTemporalMarker(19.5, ()-> {
                    rob.coneManipulator.setPosition(250);
                })
                .addTemporalMarker(19.6, ()-> {
                    rob.coneManipulator.open();
                })
                .addTemporalMarker(20, ()-> {
                    rob.coneManipulator.setPosition(2150);
                    rob.lift.setPosition(0);
                    // rob.intake.setPower(1);
                })


                // Cycle 5

                .addTemporalMarker(20.7, ()-> {
                    rob.coneManipulator.setPosition(2680);
                })

                .addTemporalMarker(20.8, ()-> {
                    rob.coneManipulator.close();
                })

                .addTemporalMarker(21.3, ()-> {
                    rob.coneManipulator.setPosition(2300);
                })

                .addTemporalMarker(22.4, ()-> {
                    rob.lift.setPosition(Lift.highPoleBroken-75);
                    rob.coneManipulator.setPosition(850);
                    //     rob.intake.setPower(0);
                })

                .addTemporalMarker(23.45, ()-> {
                    rob.coneManipulator.setPosition(250);
                })
                .addTemporalMarker(23.55, ()-> {
                    rob.coneManipulator.open();
                })
                .addTemporalMarker(23.7, ()-> {
                    rob.coneManipulator.setPosition(2000);
                })
                .addTemporalMarker(24, ()-> {
                    rob.coneManipulator.setPosition(0);
                    rob.lift.setPosition(0);
                })


 */

                .setConstraints(SampleMecanumDrive.getVelocityConstraint(80, Math.toRadians(300), 14.2), SampleMecanumDrive.getAccelerationConstraint(80))

                // To Main Pole
                .addTemporalMarker(()-> {
                    rob.intake.setPower(1);
                    rob.coneManipulator.setPosition(850);
                    // rob.coneManipulator.setPosition(250);
                })
                .splineTo(new Vector2d(-34, -38), Math.toRadians(90))
                .splineTo(new Vector2d(-36, -6), Math.toRadians(90))

                .setReversed(false)
                .addTemporalMarker(()-> rob.lift.setPosition(Lift.highPoleBroken-75))
                .splineTo(new Vector2d(-34, -18), Math.toRadians(270))
                .setReversed(true)

                .addTemporalMarker(()-> {
                    // rob.coneManipulator.setPosition(900);
                })

                .setConstraints(SampleMecanumDrive.getVelocityConstraint(55, Math.toRadians(180), 14.2), SampleMecanumDrive.getAccelerationConstraint(55))

                .splineTo(new Vector2d(-28.5, -6.2), Math.toRadians(65))
                .addTemporalMarker(()-> rob.intake.setPower(0))

                .resetConstraints()

                // Cycle 1

                // To Cone Stack
                .setReversed(false)
                .splineTo(new Vector2d(-32, -11.5), Math.toRadians(180))
                .splineTo(new Vector2d(-59.5-distance, -12), Math.toRadians(180))
                .waitSeconds(0.5)

                .setReversed(true)
                .splineTo(new Vector2d(-50, -12), Math.toRadians(0))
                .splineTo(HighTrajectories.cycle1.vec(), HighTrajectories.cycle1.getHeading()) // -31, -4
                //.back(1)




                // Cycle 2

                // To Cone Stack
                .setReversed(false)
                .splineTo(new Vector2d(-59-distance, -12), Math.toRadians(180))
                .waitSeconds(0.5)

                .setReversed(true)
                .splineTo(new Vector2d(-50, -12), Math.toRadians(0))
                .splineTo(HighTrajectories.cycle2.vec(), HighTrajectories.cycle2.getHeading()) // -31, -4
                //.back(1.5)

                // Cycle 3

                // To Cone Stack
                .waitSeconds(0.3)
                .setReversed(false)
                .splineTo(new Vector2d(-58.5-distance, -10), Math.toRadians(180))
                .waitSeconds(0.5)

                .setReversed(true)
                .splineTo(new Vector2d(-50, -12), Math.toRadians(0))
                .splineTo(HighTrajectories.cycle3.vec(), HighTrajectories.cycle3.getHeading()) // -31, -4
                //.back(1.5)

/*

                // Cycle 4

                // To Cone Stack
                .setReversed(false)
                .splineTo(new Vector2d(-58.2-distance, -12), Math.toRadians(185))
                .waitSeconds(0.5)

                .setReversed(true)
                .splineTo(new Vector2d(-50, -12), Math.toRadians(0))
                .splineTo(HighTrajectories.cycle4.vec(), HighTrajectories.cycle4.getHeading()) // -31, -4
                //.back(1.7) //2



                // Cycle 5

                // To Cone Stack
                .setReversed(false)
                .splineTo(new Vector2d(-57.1, -12), Math.toRadians(190))
                .waitSeconds(0.5)

                .setReversed(true)
                .splineTo(new Vector2d(-50, -12), Math.toRadians(0))
                .splineTo(HighTrajectories.cycle5.vec(), HighTrajectories.cycle5.getHeading()) // -31, -4
                //.back(2)

                .addTemporalMarker(()-> {
                    rob.coneManipulator.setPosition(0);
                })

 */

                .setReversed(false)

                .setConstraints(SampleMecanumDrive.getVelocityConstraint(65, Math.toRadians(300), 14.2), SampleMecanumDrive.getAccelerationConstraint(80))//

                .waitSeconds(0.5)
                .addTemporalMarker(()-> rob.coneManipulator.setPosition(0))

                 //Middle
                .splineTo(new Vector2d(-30, -22), Math.toRadians(270))

                //Left
                //
                //.splineTo(new Vector2d(-58, -12), Math.toRadians(180))

                // Right
                //.splineTo(new Vector2d(-24, -6), Math.toRadians(0)) // Avoid High and Mid Pole
                //.forward(4) // Don't hit poles while turning
                //.splineTo(new Vector2d(-9.5, -22), Math.toRadians(270)) // End Pos

                //.addTemporalMarker(()-> rob.driveTrain.followTrajectorySequenceAsync())
                .build();

        TrajectorySequence right = rob.driveTrain.trajectorySequenceBuilder(start)
                .setReversed(true)

                .addTemporalMarker(3.5,() -> {
                    rob.coneManipulator.setPosition(250);
                    rob.coneManipulator.open();
                })
                .addTemporalMarker(3.8, ()-> {
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
                 rob.lift.setPosition(Lift.highPoleBroken-75);
                 rob.coneManipulator.setPosition(850);
                })

                .addTemporalMarker(7.8, ()-> {
                    rob.coneManipulator.setPosition(250);

                })
                .addTemporalMarker(7.9, ()-> {
                    rob.coneManipulator.open();
                })

                .addTemporalMarker(8.4, ()-> {
                    rob.coneManipulator.setPosition(1980);
                    rob.lift.setPosition(0);
                })

                // Cycle 2

                .addTemporalMarker(9.45-timeFromCycle2, ()-> {
                    rob.coneManipulator.setPosition(2350);
                })

                .addTemporalMarker(9.55-timeFromCycle2, ()-> {
                    rob.coneManipulator.close();
                })

                .addTemporalMarker(10.1-timeFromCycle2, ()-> {
                    rob.lift.setPosition(Lift.highPoleBroken-75);
                    rob.coneManipulator.setPosition(850);
                })

                .addTemporalMarker(11.7, ()-> {
                    rob.coneManipulator.setPosition(250);

                })
                .addTemporalMarker(11.8, ()-> {
                    rob.coneManipulator.open();
                })

                .addTemporalMarker(12.1, ()-> {
                    rob.coneManipulator.setPosition(2150);
                    rob.lift.setPosition(0);
                })


                // Cycle 3

                .addTemporalMarker(14.05-0.6-timeFromCycle3, ()-> {
                    rob.coneManipulator.setPosition(2500);
                })

                .addTemporalMarker(14.15-0.6-timeFromCycle3, ()-> {
                    rob.coneManipulator.close();
                })

                .addTemporalMarker(14.7-0.6-timeFromCycle3, ()-> {
                    rob.lift.setPosition(Lift.highPoleBroken-75);
                    rob.coneManipulator.setPosition(850);
                })

                .addTemporalMarker(16.3-0.6, ()-> {
                    rob.coneManipulator.setPosition(250);
                    rob.coneManipulator.open();
                })
                .addTemporalMarker(16.4-0.4, ()-> {
                    rob.coneManipulator.setPosition(2150);
                    rob.lift.setPosition(0);
                //    rob.intake.setPower(1);
                })


                /*

                // Cycle 4

                .addTemporalMarker(17.45 - timeFromCycle4, ()-> {
                    rob.coneManipulator.setPosition(2550);
                })

                .addTemporalMarker(17.55 - timeFromCycle4, ()-> {
                    rob.coneManipulator.close();
                })

                .addTemporalMarker(18.1 - timeFromCycle4, ()-> {
                    rob.lift.setPosition(Lift.highPoleBroken-75);
                    rob.coneManipulator.setPosition(850);
                   // rob.intake.setPower(0);
                })

                .addTemporalMarker(19.5, ()-> {
                    rob.coneManipulator.setPosition(250);
                })
                .addTemporalMarker(19.6, ()-> {
                    rob.coneManipulator.open();
                })
                .addTemporalMarker(20, ()-> {
                    rob.coneManipulator.setPosition(2150);
                    rob.lift.setPosition(0);
                   // rob.intake.setPower(1);
                })


                // Cycle 5

                .addTemporalMarker(20.7, ()-> {
                    rob.coneManipulator.setPosition(2680);
                })

                .addTemporalMarker(20.8, ()-> {
                    rob.coneManipulator.close();
                })

                .addTemporalMarker(21.3, ()-> {
                    rob.coneManipulator.setPosition(2300);
                })

                .addTemporalMarker(22.4, ()-> {
                    rob.lift.setPosition(Lift.highPoleBroken-75);
                    rob.coneManipulator.setPosition(850);
               //     rob.intake.setPower(0);
                })

                .addTemporalMarker(23.35, ()-> {
                    rob.coneManipulator.setPosition(250);
                })
                .addTemporalMarker(23.45, ()-> {
                    rob.coneManipulator.open();
                })
                .addTemporalMarker(23.8, ()-> {
                    rob.coneManipulator.setPosition(2000);
                })
                .addTemporalMarker(24.5, ()-> {
                    rob.coneManipulator.setPosition(0);
                    rob.lift.setPosition(0);
                })

                 */


                .setConstraints(SampleMecanumDrive.getVelocityConstraint(80, Math.toRadians(300), 14.2), SampleMecanumDrive.getAccelerationConstraint(80))

                // To Main Pole
                .addTemporalMarker(()-> {
                    rob.intake.setPower(1);
                    rob.coneManipulator.setPosition(850);
                   // rob.coneManipulator.setPosition(250);
                })
                .splineTo(new Vector2d(-34, -38), Math.toRadians(90))
                .splineTo(new Vector2d(-36, -6), Math.toRadians(90))

                .setReversed(false)
                .addTemporalMarker(()-> rob.lift.setPosition(Lift.highPoleBroken-75))
                .splineTo(new Vector2d(-34, -18), Math.toRadians(270))
                .setReversed(true)

                .addTemporalMarker(()-> {
                   // rob.coneManipulator.setPosition(900);
                })

                .setConstraints(SampleMecanumDrive.getVelocityConstraint(55, Math.toRadians(180), 14.2), SampleMecanumDrive.getAccelerationConstraint(55))

                .splineTo(new Vector2d(-28.5, -6.2), Math.toRadians(65))
                .addTemporalMarker(()-> rob.intake.setPower(0))

                .resetConstraints()

                // Cycle 1

                // To Cone Stack
                .setReversed(false)
                .splineTo(new Vector2d(-32, -11.5), Math.toRadians(180))
                .splineTo(new Vector2d(-59.5-distance, -12), Math.toRadians(180))
                .waitSeconds(0.5)

                .setReversed(true)
                .splineTo(new Vector2d(-50, -12), Math.toRadians(0))
                .splineTo(HighTrajectories.cycle1.vec(), HighTrajectories.cycle1.getHeading()) // -31, -4
                //.back(1)




                // Cycle 2

                // To Cone Stack
                .setReversed(false)
                .splineTo(new Vector2d(-59-distance, -12), Math.toRadians(180))
                .waitSeconds(0.5)

                .setReversed(true)
                .splineTo(new Vector2d(-50, -12), Math.toRadians(0))
                .splineTo(HighTrajectories.cycle2.vec(), HighTrajectories.cycle2.getHeading()) // -31, -4
                //.back(1.5)

                // Cycle 3

                // To Cone Stack
                .waitSeconds(0.3)
                .setReversed(false)
                .splineTo(new Vector2d(-58.5-distance, -10), Math.toRadians(180))
                .waitSeconds(0.5)

                .setReversed(true)
                .splineTo(new Vector2d(-50, -12), Math.toRadians(0))
                .splineTo(HighTrajectories.cycle3.vec(), HighTrajectories.cycle3.getHeading()) // -31, -4
                //.back(1.5)

/*

                // Cycle 4

                // To Cone Stack
                .setReversed(false)
                .splineTo(new Vector2d(-58.2-distance, -12), Math.toRadians(185))
                .waitSeconds(0.5)

                .setReversed(true)
                .splineTo(new Vector2d(-50, -12), Math.toRadians(0))
                .splineTo(HighTrajectories.cycle4.vec(), HighTrajectories.cycle4.getHeading()) // -31, -4
                //.back(1.7) //2



                // Cycle 5

                // To Cone Stack
                .setReversed(false)
                .splineTo(new Vector2d(-57.1, -12), Math.toRadians(190))
                .waitSeconds(0.5)

                .setReversed(true)
                .splineTo(new Vector2d(-50, -12), Math.toRadians(0))
                .splineTo(HighTrajectories.cycle5.vec(), HighTrajectories.cycle5.getHeading()) // -31, -4
                //.back(2)

                .addTemporalMarker(()-> {
                    rob.coneManipulator.setPosition(0);
                })


 */
                .setReversed(false)

                .setConstraints(SampleMecanumDrive.getVelocityConstraint(65, Math.toRadians(300), 14.2), SampleMecanumDrive.getAccelerationConstraint(80))//

                .waitSeconds(0.5)
                .addTemporalMarker(()-> rob.coneManipulator.setPosition(0))

                // Middle
                //.splineTo(new Vector2d(-30, -22), Math.toRadians(270))

                 //Left
                //
                //.splineTo(new Vector2d(-58, -12), Math.toRadians(180))

                // Right
                .splineTo(new Vector2d(-24, -6), Math.toRadians(0)) // Avoid High and Mid Pole
                .forward(4) // Don't hit poles while turning
                .splineTo(new Vector2d(-13.5, -22), Math.toRadians(270)) // End Pos

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
            telemetry.addLine(rob.rearCamera.getTelemetry());
            telemetry.addData("pos", rob.rearCamera.getSleeveLocation().toString());
            telemetry.addData("Time Since Init", Color.LIME.format(timer.currentSeconds()));
            telemetry.update();

            rob.update();

            location = rob.rearCamera.getSleeveLocation();

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
            default:
                rob.driveTrain.followTrajectorySequenceAsync(middle);
                break;
        }

        //rob.driveTrain.followTrajectorySequenceAsync(right);

        waitForStart();

        timer.reset();

        rob.rearCamera.camera.stopStreaming();
        rob.rearCamera.camera.closeCameraDevice();

        while(opModeIsActive() && !isStopRequested()) {
            rob.driveTrain.update();
            rob.update();


            if(!rob.driveTrain.isBusy()) {
                rob.driveTrain.setMotorPowers(0, 0,0, 0);

                stop();
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
