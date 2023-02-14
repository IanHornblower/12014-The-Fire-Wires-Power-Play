package org.firewires.meepmeeptesting;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.noahbres.meepmeep.MeepMeep;
import com.noahbres.meepmeep.roadrunner.DefaultBotBuilder;
import com.noahbres.meepmeep.roadrunner.entity.RoadRunnerBotEntity;

public class FarPoleCycle {
    public static void main(String[] args) {
        MeepMeep meepMeep = new MeepMeep(800);

        RoadRunnerBotEntity myBot = new DefaultBotBuilder(meepMeep)
                // Set bot constraints: maxVel, maxAccel, maxAngVel, maxAngAccel, track width
                .setConstraints(50, 50, Math.toRadians(180), Math.toRadians(180), 5)
                .setDimensions(16, 16)
                .followTrajectorySequence(drive ->
                        drive.trajectorySequenceBuilder(new Pose2d(-15, -62, Math.toRadians(-90)))
                                .setReversed(true)
                                .splineTo(new Vector2d(-12.5, -46), Math.toRadians(90))
                                .splineTo(new Vector2d(-12, -11), Math.toRadians(90))

                                .setReversed(false)
                                .splineTo(new Vector2d(-4, -20), Math.toRadians(-45))

                                // Cycle 1

                                .setReversed(true)
                                .splineTo(new Vector2d(-18, -11.5), Math.toRadians(180))
                                .splineTo(new Vector2d(-62.5, -11.5), Math.toRadians(180))

                                .setReversed(false)
                                .splineTo(new Vector2d(-18, -11.5), Math.toRadians(0))
                                .splineTo(new Vector2d(-4, -20), Math.toRadians(-45))

                                // Cycle 2

                                .setReversed(true)
                                .splineTo(new Vector2d(-18, -11.5), Math.toRadians(180))
                                .splineTo(new Vector2d(-62.5, -11.5), Math.toRadians(180))

                                .setReversed(false)
                                .splineTo(new Vector2d(-18, -11.5), Math.toRadians(0))
                                .splineTo(new Vector2d(-4, -20), Math.toRadians(-45))

                                // Cycle 3

                                .setReversed(true)
                                .splineTo(new Vector2d(-18, -11.5), Math.toRadians(180))
                                .splineTo(new Vector2d(-62.5, -11.5), Math.toRadians(180))

                                .setReversed(false)
                                .splineTo(new Vector2d(-18, -11.5), Math.toRadians(0))
                                .splineTo(new Vector2d(-4, -20), Math.toRadians(-45))

                                // Cycle 4

                                .setReversed(true)
                                .splineTo(new Vector2d(-18, -11.5), Math.toRadians(180))
                                .splineTo(new Vector2d(-62.5, -11.5), Math.toRadians(180))

                                .setReversed(false)
                                .splineTo(new Vector2d(-18, -11.5), Math.toRadians(0))
                                .splineTo(new Vector2d(-4, -20), Math.toRadians(-45))

                                // Cycle 5

                                .setReversed(true)
                                .splineTo(new Vector2d(-18, -11.5), Math.toRadians(180))
                                .splineTo(new Vector2d(-62.5, -11.5), Math.toRadians(180))

                                .setReversed(false)
                                .splineTo(new Vector2d(-18, -11.5), Math.toRadians(0))
                                .splineTo(new Vector2d(-4, -20), Math.toRadians(-45))

                                // Park

                                .setReversed(true)
                                .splineTo(new Vector2d(-18, -11.5), Math.toRadians(180))
                                .splineTo(new Vector2d(-60, -11.5), Math.toRadians(180)) // LEFT
                                //.splineTo(new Vector2d(-32, -11.5), Math.toRadians(180)) // MIDDLE
                                //.splineTo(new Vector2d(-6, -11.5), Math.toRadians(180)) // RIGHT

                                .build()
                );

        meepMeep.setBackground(MeepMeep.Background.FIELD_POWERPLAY_OFFICIAL)
                .setDarkMode(true)
                .setBackgroundAlpha(0.95f)
                .addEntity(myBot)
                .start();
    }
}