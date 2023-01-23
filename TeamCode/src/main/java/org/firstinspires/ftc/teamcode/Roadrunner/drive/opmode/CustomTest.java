package org.firstinspires.ftc.teamcode.Roadrunner.drive.opmode;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDrive;

/*
 * This is an example of a more complex path to really test the tuning.
 */
@Autonomous(group = "drive")
public class CustomTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        SampleMecanumDrive drive = new SampleMecanumDrive(hardwareMap);

        drive.setPoseEstimate(new Pose2d(-33, -62, Math.toRadians(-90)));

        waitForStart();

        if (isStopRequested()) return;

        Trajectory traj = drive.trajectoryBuilder(new Pose2d(-33, -62, Math.toRadians(-90)), true)
                .splineTo(new Vector2d(-30, -12), Math.toRadians(62))
                .build();

        drive.followTrajectoryAsync(traj);
    }
}
