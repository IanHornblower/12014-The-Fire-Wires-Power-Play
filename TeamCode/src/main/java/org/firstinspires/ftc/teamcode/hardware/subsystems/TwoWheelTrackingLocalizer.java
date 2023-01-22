package org.firstinspires.ftc.teamcode.hardware.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.TwoTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.interfaces.Subsystem;
import org.firstinspires.ftc.teamcode.math.Point;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.util.Encoder;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *
 *    ^
 *    |
 *    | ( x direction)
 *    |
 *    v
 *    <----( y direction )---->
 *        (forward)
 *    /--------------\
 *    |     ____     |
 *    |     ----     |    <- Perpendicular Wheel
 *    | ||           |
 *    | ||           |    <- Parallel Wheel
 *    |              |
 *    |              |
 *    \--------------/
 *
 */
@Config
public class TwoWheelTrackingLocalizer extends TwoTrackingWheelLocalizer implements Subsystem {
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 0.688975; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double PARALLEL_X = 0.125; // X is the up and down direction
    public static double PARALLEL_Y = 6.5625; // Y is the strafe direction

    public static double PERPENDICULAR_X = 4.4375;
    public static double PERPENDICULAR_Y = 2.875;

    // Parallel/Perpendicular to the forward axis
    // Parallel wheel is parallel to the forward axis
    // Perpendicular is perpendicular to the forward axis
    private Encoder parallelEncoder, perpendicularEncoder;
    public CRServo leftRetract, rightRetract, latteralRetract;

    private Robot drive;


    double accumulatedDistance = 0;

    Point previousPoint;

    public TwoWheelTrackingLocalizer(HardwareMap hardwareMap, Robot drive) {
        super(Arrays.asList(
            new Pose2d(PARALLEL_X, PARALLEL_Y, 0),
            new Pose2d(PERPENDICULAR_X, PERPENDICULAR_Y, Math.toRadians(90))
        ));

        this.drive = drive;

        parallelEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "fr"));
        perpendicularEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "fl"));

        perpendicularEncoder.setDirection(Encoder.Direction.REVERSE);
        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)

        leftRetract = drive.hwMap.get(CRServo.class, "leftRe");
        rightRetract = drive.hwMap.get(CRServo.class, "rightRe");
        latteralRetract = drive.hwMap.get(CRServo.class, "latRe");
        latteralRetract.setDirection(DcMotorSimple.Direction.REVERSE);
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @Override
    public double getHeading() {
        return drive.imu.getRawExternalHeading();
    }

    @Override
    public Double getHeadingVelocity() {
        return drive.getExternalHeadingVelocity();
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(parallelEncoder.getCurrentPosition()),
                encoderTicksToInches(perpendicularEncoder.getCurrentPosition())
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(
                encoderTicksToInches(parallelEncoder.getCorrectedVelocity()),
                encoderTicksToInches(perpendicularEncoder.getCorrectedVelocity())
        );
    }

    public Pose2D getPose() {
        Pose2d pose = getPoseEstimate();

        return new Pose2D(-pose.getY(), pose.getX(), pose.getHeading());
    }

    public Pose2D getRawVelocityPos() {
        Pose2d pose = getPoseVelocity();

        if(pose != null) {
            return new Pose2D(-pose.getY(), pose.getX(), pose.getHeading());
        }
        return new Pose2D(0, 0, 0);
    }

    public Pose2D getRotatedVelocityPos() {
        return getRawVelocityPos().rotate(-getPose().heading);
    }

    @Override
    public void update() {
        super.update();

        accumulatedDistance += getPose()
                .toPoint()
                .subtract(previousPoint)
                .hypot();

        previousPoint = getPose().toPoint();

        TelemetryPacket robotPosition = new TelemetryPacket();

        Point point = new Point(getPoseEstimate().getX() + 9, getPoseEstimate().getY()).rotate(getPoseEstimate().getHeading());
        robotPosition.fieldOverlay().strokeCircle(getPoseEstimate().getX(), getPoseEstimate().getY(), 9);
        robotPosition.fieldOverlay().strokeLine(getPoseEstimate().getX(), getPoseEstimate().getY(), point.x, point.y);

        FtcDashboard.getInstance().sendTelemetryPacket(robotPosition);
    }

    public double getAccumulatedDistance() {
        return accumulatedDistance;
    }

    public void resetAccumulatedDistance() {
        accumulatedDistance = 0;
    }

    public void setStartPosition(Pose2D pose) {
        setPoseEstimate(new Pose2d(pose.y, pose.x, pose.heading));
    }

    @Override
    public void init() throws InterruptedException {
        previousPoint = getPose().toPoint();
    }
}