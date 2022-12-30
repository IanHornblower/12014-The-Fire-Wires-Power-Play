package org.firstinspires.ftc.teamcode.OpModes.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Transform2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.spartronics4915.lib.T265Camera;

import org.firstinspires.ftc.teamcode.hardware.Robot;

@Disabled
@TeleOp(name="Test T265", group="Iterative Opmode")
public class TestCameraOpModeRpboogfd extends OpMode
{
    // We treat this like a singleton because there should only ever be one object per camera
    private static T265Camera slamra = null;

    Translation2d translation;
    Rotation2d rotation;
    T265Camera.CameraUpdate up;

    Robot rob;

    private final FtcDashboard dashboard = FtcDashboard.getInstance();

    @Override
    public void init() {
        if (slamra == null) {
            slamra = new T265Camera(new Transform2d(), 0.0001, hardwareMap.appContext);
        }

        slamra.setPose(new Pose2d(0, 0, new Rotation2d(0)));

        rob = new Robot(hardwareMap, telemetry, Robot.OPMODE_TYPE.TELEOP);
        telemetry = rob.getTelemetry();
        try {
            rob.init();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        slamra.start();
    }

    @Override
    public void loop() {
        final int robotRadius = 9; // inches

        rob.driveTrain.setWeightedDrivePower(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);

        telemetry.addData("acc distance", rob.t265.accumulatedDistance);
        telemetry.update();

        try {
            rob.update();
        } catch (Exception e) {
            e.printStackTrace();
        }

        TelemetryPacket packet = new TelemetryPacket();
        Canvas field = packet.fieldOverlay();







        up = slamra.getLastReceivedCameraUpdate();
        if (up == null) return;

        // We divide by 0.0254 to convert meters to inches
        translation = new Translation2d(up.pose.getTranslation().getX() / 0.0254, up.pose.getTranslation().getY() / 0.0254);
        rotation = up.pose.getRotation();











        field.strokeCircle(translation.getX(), translation.getY(), robotRadius);
        double arrowX = rotation.getCos() * robotRadius, arrowY = rotation.getSin() * robotRadius;
        double x1 = translation.getX() + arrowX  / 2, y1 = translation.getY() + arrowY / 2;
        double x2 = translation.getX() + arrowX, y2 = translation.getY() + arrowY;
        field.strokeLine(x1, y1, x2, y2);

        packet.addLine("Pose: " + up.pose.toString());

        dashboard.sendTelemetryPacket(packet);
    }

    @Override
    public void stop() {
        slamra.stop();
    }

}