package org.firstinspires.ftc.teamcode.hardware.subsystems;

import static org.firstinspires.ftc.teamcode.util.AdditonalUtils.ConvertPose2DtoPose2d;

import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.hardware.interfaces.Subsystem;
import org.firstinspires.ftc.teamcode.math.Pose2D;

import java.util.function.Supplier;

public class T265Ex implements Subsystem {
    T265Camera camera;
    T265Camera.CameraUpdate update;

    private HardwareMap hardwareMap;
    private Supplier<Pose2D> odomPose;
    private double covariance;

    private Pose2D startPosition = null;
    public Pose2D pose;

    public T265Ex(HardwareMap hardwareMap) {
        this.hardwareMap = hardwareMap;
        this.odomPose = null;
    }

    public T265Ex(HardwareMap hardwareMap, Supplier<Pose2D> odomPose, double covariance) {
        this.hardwareMap = hardwareMap;
        this.odomPose = odomPose;
    }

    public void setStartPosition(Pose2D start) {
        startPosition = start;
    }

    // Might have to change first argument in T265Camera to the actual transformation from center of robot unsure
    @Override
    public void init() throws InterruptedException {
        if(camera == null) {
            if(odomPose != null) {
                camera = new T265Camera(new Transform2d(), covariance, hardwareMap.appContext);
            }
            else {
                camera = new T265Camera(new Transform2d(), 1e-6, hardwareMap.appContext);

            }
        }

        if(startPosition == null) {
            throw new InterruptedException("No StartPosition Defined");
        }

        pose = startPosition;

        camera.setPose(ConvertPose2DtoPose2d(startPosition));
        camera.start();
    }

    @Override
    public void update() throws InterruptedException { // Should only be called in active OpMode
        if(odomPose != null) {
            camera.sendOdometry(odomPose.get().x, getPose().y); //TODO: Check what needs to be sent Velocity or Position ?
        }

        update = camera.getLastReceivedCameraUpdate();
        if (update == null) return;

        pose = new Pose2D(update.pose.getX(), update.pose.getY(), update.pose.getHeading());
    }

    public void shutdown() {
        camera.stop();
    }

    public TelemetryPacket drawRobot(TelemetryPacket tp, double radius) {
        Canvas field = tp.fieldOverlay();

        field.strokeCircle(getPose().x, getPose().y, radius);
        double arrowX = Math.cos(getPose().heading) * radius, arrowY = Math.sin(getPose().heading) * radius;
        double x1 = getPose().x + arrowX  / 2, y1 = getPose().y + arrowY / 2;
        double x2 = getPose().x + arrowX, y2 = getPose().y + arrowY;
        field.strokeLine(x1, y1, x2, y2);

        return tp;
    }

    public TelemetryPacket writePosition(TelemetryPacket tp) {
        tp.addLine("Pose: " + getPose().toString());

        return tp;
    }

    public TelemetryPacket updatePacket(TelemetryPacket tp, double radius) {
        return writePosition(drawRobot(tp, radius));
    }

    public Pose2D getRawPose() {
        return pose;
    }

    public Pose2D getPose() {
        return pose.div(0.0254); // Convert M to IN
    }
}
