package org.firstinspires.ftc.teamcode.ActionSystem.actions.Depricatted;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.ActionSystem.Action;
import org.firstinspires.ftc.teamcode.control.PurePursuit;
import org.firstinspires.ftc.teamcode.control.Trajectory;
import org.firstinspires.ftc.teamcode.control.DashboardDrawUtil;
import org.firstinspires.ftc.teamcode.math.Point;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.util.Timer;

@Config
public class FollowTrajectory extends Action {

    Robot robot;
    Trajectory trajectory;
    double radius;

    Timer timer = new Timer();

    public FollowTrajectory(Robot robot, Trajectory trajectory, double radius) {
        this.robot = robot;
        this.trajectory = trajectory;
        this.radius = radius;
    }

    Trajectory extendedPath;

    @Override
    public void startAction() {
        FtcDashboard.getInstance().sendTelemetryPacket(DashboardDrawUtil.drawTrajectory(trajectory, new TelemetryPacket()));

        extendedPath = new Trajectory(PurePursuit.extendPath(trajectory.getPath(), radius));
        timer.start();
    }

    public static double turnP = 0.06;
    public static double forwardP = 0.06;

    // Suplement the solute of F & T from P Control Loop to Velo Graph

    @Override
    public void runAction() throws InterruptedException {
        robot.update();
        Pose2D pose = robot.t265.getPose();
        Point lookAhead = PurePursuit.getLookAheadPoint(trajectory.getPath(), pose, radius);

        Point error = new Point(lookAhead.x - pose.x, lookAhead.y - pose.y);
        double theta = error.atan2();
        double distance = error.hypot();

        double f = distance * forwardP;
        double t = normalize(theta - pose.heading) * turnP;

        f *= Math.cos(
                Range.clip(
                        normalize(theta - pose.heading),
                        -Math.PI/2, Math.PI/2)
        );

        double left = f + t;
        double right = f - t;

        robot.driveTrain.setMotorPowers(left, right);

        isComplete = Math.abs(error.x + error.y) < 2;
    }

    @Override
    public void stopAction() {
        FtcDashboard.getInstance().clearTelemetry();
        robot.driveTrain.stopDriveTrain();
    }

    private static double normalize(double radians) {
        if (radians >= Math.PI) radians -= Math.PI * 2.0;
        if (radians < -Math.PI) radians += Math.PI * 2.0;
        return radians;
    }

}
