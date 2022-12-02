package org.firstinspires.ftc.teamcode.control;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import org.firstinspires.ftc.teamcode.math.Point;

public class TrajectoryDrawUtil {

    public static int strokeWidth = 1;
    public static String color = "lime";

    // This ik works
    public static TelemetryPacket drawTrajectory(Trajectory trajectory, TelemetryPacket telemetryPacket) {
        telemetryPacket.fieldOverlay().setStroke(color);
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
}
