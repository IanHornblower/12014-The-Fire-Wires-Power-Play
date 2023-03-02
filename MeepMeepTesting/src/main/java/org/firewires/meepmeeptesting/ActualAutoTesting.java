package org.firewires.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class ActualAutoTesting {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 15)
                .setDimensions(16, 16)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-39, 62, Math.toRadians(90)))
                                .setReversed(true)
                                .splineTo(new Vector2d(-34, 38), Math.toRadians(270))
                                .splineTo(new Vector2d(-36, 6), Math.toRadians(270))

                                .setReversed(false)
                               // .addTemporalMarker(()-> rob.lift.setPosition(Lift.highPoleBroken-75))
                                .splineTo(new Vector2d(-34, 18), Math.toRadians(90))
                                .setReversed(true)

                                .addTemporalMarker(()-> {
                                    // rob.coneManipulator.setPosition(900);
                                })

                                //.setConstraints(SampleMecanumDrive.getVelocityConstraint(55, Math.toRadians(180), 14.2), SampleMecanumDrive.getAccelerationConstraint(55))

                                .splineTo(new Vector2d(-27.5, 6.2), Math.toRadians(-60))
//                                .addTemporalMarker(()-> rob.intake.setPower(0))

                                .resetConstraints()

                                // Cycle 1

                                // To Cone Stack
                                .setReversed(false)
                                .splineTo(new Vector2d(-32, 11.5), Math.toRadians(180))
                                .splineTo(new Vector2d(-58.2, 12), Math.toRadians(180))
                                .waitSeconds(0.5)

                                .setReversed(true)
                                .splineTo(new Vector2d(-50, 12), Math.toRadians(0))
                                .splineTo(HighTrajectories.cycle1.vec(), HighTrajectories.cycle1.getHeading()) // -31, -4
                                .back(1)




                                // Cycle 2

                                // To Cone Stack
                                .setReversed(false)
                                .splineTo(new Vector2d(-57.7, 12), Math.toRadians(180))
                                .waitSeconds(0.5)

                                .setReversed(true)
                                .splineTo(new Vector2d(-50, 12), Math.toRadians(0))
                                .splineTo(HighTrajectories.cycle2.vec(), HighTrajectories.cycle2.getHeading()) // -31, -4
                                .back(1.5)

                                // Cycle 3

                                // To Cone Stack
                                .waitSeconds(0.3)
                                .setReversed(false)
                                .splineTo(new Vector2d(-57.7, 12), Math.toRadians(180))
                                .waitSeconds(0.5)

                                .setReversed(true)
                                .splineTo(new Vector2d(-50, 12), Math.toRadians(0))
                                .splineTo(HighTrajectories.cycle3.vec(), HighTrajectories.cycle3.getHeading()) // -31, -4
                                .back(1.5)



                                // Cycle 4

                                // To Cone Stack
                                .setReversed(false)
                                .splineTo(new Vector2d(-57.7, 12), Math.toRadians(175))
                                .waitSeconds(0.5)

                                .setReversed(true)
                                .splineTo(new Vector2d(-50, 12), Math.toRadians(0))
                                .splineTo(HighTrajectories.cycle4.vec(), HighTrajectories.cycle4.getHeading()) // -31, -4
                                .back(1.7) //2



                                // Cycle 5

                                // To Cone Stack
                                .setReversed(false)
                                .splineTo(new Vector2d(-57.1, 12), Math.toRadians(170))
                                .waitSeconds(0.5)

                                .setReversed(true)
                                .splineTo(new Vector2d(-50, 12), Math.toRadians(0))
                                .splineTo(HighTrajectories.cycle5.vec(), HighTrajectories.cycle5.getHeading()) // -31, -4
                                .back(2)

                                .addTemporalMarker(()-> {
                                 //   rob.coneManipulator.setPosition(0);
                                })

                                .setReversed(false)

                                //.setConstraints(SampleMecanumDrive.getVelocityConstraint(65, Math.toRadians(300), 14.2), SampleMecanumDrive.getAccelerationConstraint(80))//

                                .waitSeconds(0.5)

                                // Middle
                                //.splineTo(new Vector2d(-30, 22), Math.toRadians(90))

                                //Left
                                //
                                //.splineTo(new Vector2d(-58, 12), Math.toRadians(180))

                                // Right
                                .splineTo(new Vector2d(-24, 6), Math.toRadians(0)) // Avoid High and Mid Pole
                                .forward(4) // Don't hit poles while turning
                                .splineTo(new Vector2d(-9.5, 22), Math.toRadians(90)) // End Pos

                                //.addTemporalMarker(()-> rob.driveTrain.followTrajectorySequenceAsync())
                                .build());

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}