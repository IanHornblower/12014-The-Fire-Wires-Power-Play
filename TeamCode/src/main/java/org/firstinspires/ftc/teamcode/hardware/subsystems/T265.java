package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.interfaces.Subsystem;
import org.firstinspires.ftc.teamcode.math.Point;
import org.firstinspires.ftc.teamcode.math.Pose2D;

public class T265 implements Subsystem {

    private static T265Camera slamra = null;
    T265Camera.CameraUpdate up;
    Pose2d startPose = new Pose2d(0, 0, new Rotation2d(0));

    Robot robot;
    double robotRadius = 8;

    Point lastPoint = new Point(0, 0);
    public double accumulatedDistance = 0;

    public T265(Robot robot) {
        this.robot = robot;
    }

    public void setStartPosition(Pose2D pose) {
        startPose = new Pose2d(pose.x, pose.y, new Rotation2d(pose.heading));
    }

    public void resetAccumulatedDistance() {
        accumulatedDistance = 0;
    }

    public Pose2D getPose() {
        if(up != null) {
            return new Pose2D(
                    up.pose.getTranslation().getX(),
                    up.pose.getTranslation().getY(),
                    up.pose.getRotation().getRadians()
            ).div(0.0254).rotate(Math.toRadians(180));
        }
        else {
            return new Pose2D(0, 0,0);
        }
    }

    public double getAccumulatedDistance() {
        return accumulatedDistance;
    }

    @Override
    public void init() throws InterruptedException {
        //slamra = new T265Camera(
        //        new Transform2d(new Translation2d(0.1610868, 0.083566), new Rotation2d(0)),
        //        0.0001, robot.hwMap.appContext);

        slamra = new T265Camera(
                new Transform2d(),
                0.0001, robot.hwMap.appContext);

        slamra.setPose(startPose);


    }

    public void start() {
        slamra.start();
    }

    public void stop() {
        slamra.stop();
    }

    @Override
    public void update() throws InterruptedException {
        up = slamra.getLastReceivedCameraUpdate();
        if (up == null) return;

        accumulatedDistance += getPose().getDistanceFrom(lastPoint);
        lastPoint = getPose().toPoint();

        // Draw on the Dashboard Field

        TelemetryPacket packet = new TelemetryPacket();

        packet.fieldOverlay().strokeCircle(getPose().x, getPose().y, robotRadius);
        double arrowX = Math.cos(getPose().heading) * robotRadius, arrowY = Math.sin(getPose().heading) * robotRadius;
        double x1 = getPose().x + arrowX  / 2, y1 = getPose().y + arrowY / 2;
        double x2 = getPose().x + arrowX, y2 = getPose().y + arrowY;
        packet.fieldOverlay().strokeLine(x1, y1, x2, y2);

        robot.FtcDashboardInstance.sendTelemetryPacket(packet);
    }
}
