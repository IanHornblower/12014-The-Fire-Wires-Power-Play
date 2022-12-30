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


public class FollowActualTrajectory extends Action {

    // Action Stuff
    Robot robot;
    DriveTrain dt;
    Trajectory trajectory;

    // Follower Stuff
    ArrayList<Pose2D> path;
    double[] lengths;
    int lastIndex;
    Pose2D lastPoint;

    BasicPID xPID = new BasicPID(DriveConstraints.xPID);
    BasicPID yPID = new BasicPID(DriveConstraints.yPID);
    BasicPID headingPID = new BasicPID(DriveConstraints.headingPID);
    AngleController headingController = new AngleController(headingPID);
    BasicPID distanceController = new BasicPID(DriveConstraints.trajectoryPID);

    public FollowActualTrajectory(Robot robot, Trajectory trajectory) {
        this.robot = robot;
        this.dt = robot.driveTrain;
        this.trajectory = trajectory;
    }

    @Override
    public void startAction() {
        robot.t265.resetAccumulatedDistance();
        path = trajectory.getPath();

        lengths = Trajectory.getSegmentLengths(path);

        lastIndex = path.size()-1;
        lastPoint = path.get(lastIndex);
    }

    // Start at first point
    int index = 1;

    double xP, yP, distanceP, angle, headingP;

    @Override
    public void runAction() throws InterruptedException {
        Pose2D pos = robot.t265.getPose();

        double error = Math.abs(pos.getDistanceFrom(lastPoint));

        if(robot.t265.accumulatedDistance > lengths[index-1]) {
            robot.t265.resetAccumulatedDistance();
            index++;
        }

        if(index == path.size()-2) {
            headingP = headingController.calculate(path.get(index).heading, pos.heading);

            xP = xPID.calculate(path.get(index).x, pos.x);
            yP = yPID.calculate(path.get(index).y, pos.y);
        }
        else {
            headingP = headingController.calculate(path.get(index).heading, pos.heading);

            distanceP = distanceController.calculate(error, 0);
            angle = Math.atan2(yPID.calculate(path.get(index).y, pos.y), xPID.calculate(path.get(index).x, pos.x));
            xP = Math.cos(angle) * distanceP;
            yP = Math.sin(angle) * distanceP;
        }

        dt.driveFieldCentric(xP, yP, headingP);

        if(error < 2) isComplete = true;
    }

    @Override
    public void stopAction() {
        dt.stopDriveTrain();
    }
}
