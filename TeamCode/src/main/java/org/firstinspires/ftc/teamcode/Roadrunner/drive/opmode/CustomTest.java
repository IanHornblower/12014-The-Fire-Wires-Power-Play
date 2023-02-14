package org.firstinspires.ftc.teamcode.Roadrunner.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDrive;

import java.io.PipedOutputStream;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class CustomTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        Pose2d start = new Pose2d(-39, -62, Math.toRadians(-90));

        drive.setPoseEstimate(start);

        waitForStart();

        if (isStopRequested()) return;

        Trajectory traj = drive.trajectoryBuilder(start, true)
                .splineTo(new Vector2d(-33, -38), Math.toRadians(90))
                .splineTo(new Vector2d(-28, -6), Math.toRadians(45))
                .build();

        Trajectory traj1 =  drive.trajectoryBuilder(traj.end(), false)
                .splineTo(new Vector2d(-62.5, -10), Math.toRadians(173))
                .build();

        Trajectory traj2 =  drive.trajectoryBuilder(traj1.end(), true)
                .splineTo(new Vector2d(-39, -11.5), Math.toRadians(0))
                .splineTo(new Vector2d(-28, -8), Math.toRadians(45))
                .build();

        drive.followTrajectory(traj);
        sleep(1000);
        drive.followTrajectory(traj1);
        sleep(1000);
        drive.followTrajectory(traj2);
        sleep(1000);
    }
}
