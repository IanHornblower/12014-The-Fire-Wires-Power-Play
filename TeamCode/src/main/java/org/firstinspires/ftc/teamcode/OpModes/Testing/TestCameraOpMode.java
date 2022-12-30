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
@TeleOp(name="Test T265 with Robot", group="Iterative Opmode")
public class TestCameraOpMode extends OpMode
{
    Robot rob;

    @Override
    public void init() {
        rob = new Robot(hardwareMap, telemetry, Robot.OPMODE_TYPE.TELEOP);
        try {
            rob.init();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        telemetry = rob.getTelemetry();
    }

    @Override
    public void init_loop() {
    }

    @Override
    public void start() {
        rob.t265.start();
    }

    @Override
    public void loop() {
        rob.driveTrain.setWeightedDrivePower(gamepad1.left_stick_x, -gamepad1.left_stick_y, gamepad1.right_stick_x);

        telemetry.addData("Pose", rob.t265.getPose());
        telemetry.addData("Robot Acc Dist", rob.t265.accumulatedDistance);
        telemetry.update();

        try {
            rob.update();
        } catch (InterruptedException e) {
            e.printStackTrace();
        }
    }

    @Override
    public void stop() {
        rob.t265.stop();
    }

}