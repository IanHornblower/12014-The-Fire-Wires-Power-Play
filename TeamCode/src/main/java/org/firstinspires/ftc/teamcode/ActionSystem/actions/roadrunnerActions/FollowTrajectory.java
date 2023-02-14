package org.firstinspires.ftc.teamcode.ActionSystem.actions.roadrunnerActions;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.AngleController;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.roadrunner.drive.DriveSignal;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.ActionSystem.Action;
import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.DriveConstraints;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.util.Timer;

public class FollowTrajectory extends Action {

    Robot robot;
    SampleMecanumDrive dt;

    Trajectory trajectory;
    public FollowTrajectory(Robot robot, Trajectory trajectory) {
        this.robot = robot;
        this.dt = robot.driveTrain;
        this.trajectory = trajectory;
    }

    @Override
    public void startAction() {
        dt.followTrajectoryAsync(trajectory);
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
