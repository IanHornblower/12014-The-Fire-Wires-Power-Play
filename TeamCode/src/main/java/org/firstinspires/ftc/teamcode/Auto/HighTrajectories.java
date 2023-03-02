package org.firstinspires.ftc.teamcode.Auto;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.TrajectoryBuilder;

import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Lift;

public class HighTrajectories {
    public static Pose2d cycle1 = new Pose2d(-29.7-1, -4.8-1, Math.toRadians(68));

    public static Pose2d cycle2 = new Pose2d(-29.7-1, -4.8-1, Math.toRadians(65));

    public static Pose2d cycle3 = new Pose2d(-29.7-1, -4.8-1, Math.toRadians(65));

    public static Pose2d cycle4 = new Pose2d(-29.7-1, -4.8-1, Math.toRadians(65));

    public static Pose2d cycle5 = new Pose2d(-29.7-1, -4.8-1, Math.toRadians(65));

    // FIX PRELOAD
    // FIX 4th cycle and finish 5th
    // DO Mid Auto

    public static Pose2d cycle1r = new Pose2d(-31.7, 6.3, Math.toRadians(-60));

    public static Pose2d cycle2r = new Pose2d(-32.3, 5.8, Math.toRadians(-47));

    public static Pose2d cycle3r = new Pose2d(-32.3, 5.8, Math.toRadians(-47));

    public static Pose2d cycle4r = new Pose2d(-32.5, 6, Math.toRadians(-45));

    public static Pose2d cycle5r = new Pose2d(-32.5, 5.8, Math.toRadians(-56));

    Robot rob;

    Pose2d start = new Pose2d(-39, -62, Math.toRadians(-90));

    public HighTrajectories(Robot rob) {
        this.rob = rob;
    }

}
