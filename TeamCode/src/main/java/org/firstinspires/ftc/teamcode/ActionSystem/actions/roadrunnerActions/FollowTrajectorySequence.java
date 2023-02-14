package org.firstinspires.ftc.teamcode.ActionSystem.actions.roadrunnerActions;

import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.ActionSystem.Action;
import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.Roadrunner.trajectorysequence.TrajectorySequence;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.util.Timer;

public class FollowTrajectorySequence extends Action {

    Robot robot;
    SampleMecanumDrive dt;


    TrajectorySequence trajectory;
    public FollowTrajectorySequence(Robot robot, TrajectorySequence trajectory) {
        this.robot = robot;
        this.dt = robot.driveTrain;
        this.trajectory = trajectory;
    }

    @Override
    public void startAction() {
        dt.followTrajectorySequenceAsync(trajectory);
    }

    @Override
    public void runAction() {
        isComplete = !dt.isBusy();
    }

    @Override
    public void stopAction() {
        dt.setMotorPowers(0, 0, 0,0);
    }
}
