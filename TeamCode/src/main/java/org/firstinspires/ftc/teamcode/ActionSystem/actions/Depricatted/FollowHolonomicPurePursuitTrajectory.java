package org.firstinspires.ftc.teamcode.ActionSystem.actions.Depricatted;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.AngleController;
import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;

import org.firstinspires.ftc.teamcode.ActionSystem.Action;
import org.firstinspires.ftc.teamcode.control.PurePursuit;
import org.firstinspires.ftc.teamcode.control.Trajectory;
import org.firstinspires.ftc.teamcode.hardware.DriveConstraints;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.math.Point;
import org.firstinspires.ftc.teamcode.math.Pose2D;

import java.util.ArrayList;

public class FollowHolonomicPurePursuitTrajectory extends Action {

    // Action Stuff
    DriveTrain dt;
    Trajectory trajectory;
    double radius;

    // Follower Stuff
    ArrayList<Pose2D> path;
    ArrayList<Pose2D> extendedPath;
    double[] lengths;
    int lastIndex;
    Pose2D lastPoint;
    Point pointToFollow;

    Robot robot;

    BasicPID xPID = new BasicPID(DriveConstraints.xPID);
    BasicPID yPID = new BasicPID(DriveConstraints.yPID);
    BasicPID headingPID = new BasicPID(DriveConstraints.headingPID);
    AngleController headingController = new AngleController(headingPID);

    public FollowHolonomicPurePursuitTrajectory(Robot robot, Trajectory trajectory, double radius) {
        this.dt = robot.driveTrain;
        this.robot = robot;
        this.trajectory = trajectory;
        this.radius = radius;
    }

    @Override
    public void startAction() {
        path = trajectory.getPath();
        extendedPath = PurePursuit.extendPath(path, radius);

        lengths = Trajectory.getSegmentLengths(path);

        lastIndex = path.size()-1;
        lastPoint = path.get(lastIndex);
    }

    // Start at first point
    int index = 1;

    @Override
    public void runAction() throws InterruptedException {
        Pose2D pos = robot.t265.getPose();
        pointToFollow = PurePursuit.getLookAheadPoint(extendedPath, robot.t265.getPose(), radius);

        double error = Math.abs(robot.t265.getPose().getDistanceFrom(lastPoint)) - radius;

        if(robot.t265.accumulatedDistance > lengths[index-1]) {
            robot.t265.resetAccumulatedDistance();
            index++;
        }

        double xP = xPID.calculate(pointToFollow.x, pos.x);
        double yP = yPID.calculate(pointToFollow.y, pos.y);

        double headingP = headingController.calculate(path.get(index).heading, pos.heading);

        dt.driveFieldCentric(xP, yP, headingP);

        // It funky
        //if(error < distanceTolerance && dt.getCombinedVelocity() < 0.5) isComplete = true;
        isComplete = error < 2;
    }

    @Override
    public void stopAction() {
        dt.stopDriveTrain();
    }
}
