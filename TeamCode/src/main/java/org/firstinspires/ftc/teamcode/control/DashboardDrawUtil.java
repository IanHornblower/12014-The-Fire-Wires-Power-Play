package org.firstinspires.ftc.teamcode.control;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.teamcode.math.Point;
import org.firstinspires.ftc.teamcode.math.Pose2D;

public class DashboardDrawUtil {

    public static int strokeWidth = 1;
    public static String trajectoryColor = "lime";
    public static String robotColor = "black";
    public static String trailColor = "blue";

    public static double robotRadius = 8;

    // This ik works
    public static TelemetryPacket drawTrajectory(Trajectory trajectory, TelemetryPacket telemetryPacket) {
        telemetryPacket.fieldOverlay().setStroke(trajectoryColor);
        telemetryPacket.fieldOverlay().setStrokeWidth(strokeWidth);

        for(int i = 0; i <= trajectory.getPath().size()-2; i++) {
            Point current = trajectory.getPath().get(i).toPoint().rotate(-Math.PI/2);
            Point next = trajectory.getPath().get(i+1).toPoint().rotate(-Math.PI/2);

            telemetryPacket.fieldOverlay().strokeLine(current.x, current.y, next.x, next.y);
        }

        return telemetryPacket;
    }

    // Test this ion know if it works
    public static TelemetryPacket drawAuto(Trajectory[] trajectories, TelemetryPacket telemetryPacket) {
        for(Trajectory trajectory : trajectories) {
            drawTrajectory(trajectory, telemetryPacket);
        }

        return telemetryPacket;
    }

    public static TelemetryPacket drawRobot(Pose2D robotPosition, TelemetryPacket telemetryPacket) {
        telemetryPacket.fieldOverlay().setStroke(robotColor);
        telemetryPacket.fieldOverlay().setStrokeWidth(strokeWidth);

        telemetryPacket.fieldOverlay().strokeCircle(robotPosition.x, robotPosition.y, robotRadius);

        Point orientationEnd = robotPosition.toPoint().add(new Point(0, robotRadius)).rotate(robotPosition.heading);

        telemetryPacket.fieldOverlay().strokeLine(robotPosition.x, robotPosition.y, orientationEnd.x, orientationEnd.y);

        return telemetryPacket;
    }

    //public static TelemetryPacket drawTrail(int trailCount) {
//
    //}
}
