package org.firstinspires.ftc.teamcode.hardware.subsystems;

import androidx.annotation.NonNull;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.localization.ThreeTrackingWheelLocalizer;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.interfaces.Subsystem;
import org.firstinspires.ftc.teamcode.util.Encoder;
import org.firstinspires.ftc.teamcode.util.Timer;

import java.util.Arrays;
import java.util.List;

/*
 * Sample tracking wheel localizer implementation assuming the standard configuration:
 *                 0
 *          /-------y------\
 *          |     ____     |
 *          |     ----     |
 *    90   x| ||        || |   270
 *          | ||        || |
 *          |              |
 *          |              |
 *          \--------------/
 *
 */
@Config
public class StandardTrackingWheelLocalizer extends ThreeTrackingWheelLocalizer {
    public static double TICKS_PER_REV = 8192;
    public static double WHEEL_RADIUS = 0.688975; // in
    public static double GEAR_RATIO = 1; // output (wheel) speed / input (encoder) speed

    public static double LATERAL_DISTANCE = 13.8; //13.5 // in; distance between the left and right wheels
    public static double FORWARD_OFFSET =  4.4375;  // in; offset of the lateral wheel

    public static double X_MULTIPLIER = 1.025641025641026;
    public static double Y_MULTIPLIER = 1.076233183856502;

    private Encoder leftEncoder, rightEncoder, frontEncoder;

    Robot robot;

    public StandardTrackingWheelLocalizer(Robot robot) {
        super(Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));

        this.robot = robot;

        leftEncoder = new Encoder(robot.hwMap.get(DcMotorEx.class, "fr"));  // 0
        rightEncoder = new Encoder(robot.hwMap.get(DcMotorEx.class, "bl")); // 3
        frontEncoder = new Encoder(robot.hwMap.get(DcMotorEx.class, "fl")); // 1

        //leftEncoder.setDirection(Encoder.Direction.REVERSE);
        //rightEncoder.setDirection(Encoder.Direction.REVERSE);
        frontEncoder.setDirection(Encoder.Direction.REVERSE);
        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
    }

    public StandardTrackingWheelLocalizer(HardwareMap hardwareMap) {
        super(Arrays.asList(
                new Pose2d(0, LATERAL_DISTANCE / 2, 0), // left
                new Pose2d(0, -LATERAL_DISTANCE / 2, 0), // right
                new Pose2d(FORWARD_OFFSET, 0, Math.toRadians(90)) // front
        ));

        leftEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "fr"));  // 0
        rightEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "bl")); // 3
        frontEncoder = new Encoder(hardwareMap.get(DcMotorEx.class, "fl")); // 1

        //leftEncoder.setDirection(Encoder.Direction.REVERSE);
        //rightEncoder.setDirection(Encoder.Direction.REVERSE);
        frontEncoder.setDirection(Encoder.Direction.REVERSE);
        // TODO: reverse any encoders using Encoder.setDirection(Encoder.Direction.REVERSE)
    }

    public static double encoderTicksToInches(double ticks) {
        return WHEEL_RADIUS * 2 * Math.PI * GEAR_RATIO * ticks / TICKS_PER_REV;
    }

    @NonNull
    @Override
    public List<Double> getWheelPositions() {
        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCurrentPosition() * X_MULTIPLIER),
                encoderTicksToInches(rightEncoder.getCurrentPosition() * X_MULTIPLIER),
                encoderTicksToInches(frontEncoder.getCurrentPosition() * Y_MULTIPLIER)
        );
    }

    @NonNull
    @Override
    public List<Double> getWheelVelocities() {
        // TODO: If your encoder velocity can exceed 32767 counts / second (such as the REV Through Bore and other
        //  competing magnetic encoders), change Encoder.getRawVelocity() to Encoder.getCorrectedVelocity() to enable a
        //  compensation method

        return Arrays.asList(
                encoderTicksToInches(leftEncoder.getCorrectedVelocity() * X_MULTIPLIER),
                encoderTicksToInches(rightEncoder.getCorrectedVelocity() * X_MULTIPLIER),
                encoderTicksToInches(frontEncoder.getCorrectedVelocity() * Y_MULTIPLIER)
        );
    }

}