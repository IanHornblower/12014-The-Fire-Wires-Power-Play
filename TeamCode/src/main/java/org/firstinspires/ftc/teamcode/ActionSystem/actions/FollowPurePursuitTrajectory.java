package org.firstinspires.ftc.teamcode.ActionSystem.actions;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.AngleController;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;
import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.roadrunner.util.Angle;
import com.qualcomm.robotcore.util.Range;
import org.firstinspires.ftc.teamcode.ActionSystem.Action;
import org.firstinspires.ftc.teamcode.control.AsymStuff.SqrtCoefficients;
import org.firstinspires.ftc.teamcode.control.AsymStuff.SqrtControl;
import org.firstinspires.ftc.teamcode.control.PurePursuit;
import org.firstinspires.ftc.teamcode.control.Trajectory;
import org.firstinspires.ftc.teamcode.hardware.DriveConstraints;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.math.Curve;
import org.firstinspires.ftc.teamcode.math.Point;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.util.Timer;

import java.util.ArrayList;

import static org.firstinspires.ftc.teamcode.hardware.DriveConstraints.*;

public class FollowPurePursuitTrajectory extends Action {

    // Action Stuff
    Robot robot;
    Trajectory trajectory;
    double radius;
    double timeout = 1e+9;

    // Follower Stuff
    ArrayList<Pose2D> path;
    ArrayList<Pose2D> extendedPath;
    boolean reversed = false;
    int lastIndex;
    Pose2D lastPoint;
    Point pointToFollow;

    Timer timer = new Timer();

    public FollowPurePursuitTrajectory(Robot robot, Trajectory trajectory, double radius) {
        this.robot = robot;
        this.trajectory = trajectory;
        this.radius = radius;
    }

    public FollowPurePursuitTrajectory(Robot robot, Trajectory trajectory, double radius, boolean reversed) {
        this.robot = robot;
        this.trajectory = trajectory;
        this.radius = radius;
        this.reversed = reversed;
    }

    public FollowPurePursuitTrajectory(Robot robot, Trajectory trajectory, double radius, double timeout) {
        this.robot = robot;
        this.trajectory = trajectory;
        this.radius = radius;
        this.timeout = timeout;
    }

    public FollowPurePursuitTrajectory(Robot robot, Trajectory trajectory, double radius, boolean reversed, double timeout) {
        this.robot = robot;
        this.trajectory = trajectory;
        this.radius = radius;
        this.reversed = reversed;
        this.timeout = timeout;
    }

    SqrtControl xController = new SqrtControl(new SqrtCoefficients(0.09, 0.0, 0.05));
    SqrtControl yController = new SqrtControl(new SqrtCoefficients(0.09, 0.0, 0.05));

    AngleController hController = new AngleController(new BasicPID(new PIDCoefficients(1, 0, 0)));

    Point pointToFollowPrevoius;

    @Override
    public void startAction() {
        path = trajectory.getPath();
        extendedPath = PurePursuit.extendPath(path, radius);

        lastIndex = path.size()-1;
        lastPoint = path.get(lastIndex);

        timer.start();
    }

    double tangent = 0;

    @Override
    public void runAction() throws InterruptedException {
        pointToFollow = PurePursuit.getLookAheadPoint(extendedPath, robot.localizer.getPose(), radius);
        Pose2D robotPose = robot.localizer.getPose();

        Point error = new Point(pointToFollow.x - robotPose.x, pointToFollow.y - robotPose.y);
        double distance = error.hypot();

        double forward = xController.calculate(distance, 0);

        tangent = Math.atan2(pointToFollow.y - robotPose.y, pointToFollow.x - robotPose.x) - Math.toRadians(90);

        if(reversed) forward *= -1;
        if(reversed) tangent -= Math.toRadians(180);

        robot.driveTrain.setMotorPowers(
                forward + -hController.calculate(tangent, robotPose.heading),
                forward - -hController.calculate(tangent, robotPose.heading)
        );

        System.out.println(forward);

        if((new Point(path.get(path.size()-2).x - robotPose.x, path.get(path.size()-2).y - robotPose.y).hypot()) - radius < 0.5) {
            Point power = new Point(
                    xController.calculate(path.get(path.size()-2).x, robotPose.x),
                    yController.calculate(path.get(path.size()-2).y, robotPose.y)
            );

            robot.driveTrain.driveFieldCentric(power.x, power.y, -hController.calculate(path.get(path.size()-2).heading, robotPose.heading), Math.toRadians(0));
        }

        isComplete =
                        Math.abs(path.get(path.size()-2).x - robotPose.x) < 1 &&
                        Math.abs(path.get(path.size()-2).y - robotPose.y) < 1 &&
                        Math.abs(Curve.getShortestDistance(path.get(path.size()-2).heading, robotPose.heading)) < Math.toRadians(5) &&
                        Math.abs(robot.localizer.getRawVelocityPos().heading) < Math.toRadians(12) &&
                        Math.abs(robot.localizer.getRawVelocityPos().x) < 1  &&
                        Math.abs(robot.localizer.getRawVelocityPos().y) < 1 ||
                        timer.currentSeconds() > timeout;
    }

    @Override
    public void stopAction() {
        robot.driveTrain.stopDriveTrain();
    }
}