package org.firstinspires.ftc.teamcode.OpModes.Testing;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.control.Trajectory;
import org.firstinspires.ftc.teamcode.control.DashboardDrawUtil;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.math.Point;
import org.firstinspires.ftc.teamcode.math.Pose2D;
import org.firstinspires.ftc.teamcode.util.Color;

@Disabled
@TeleOp
public class TrajectoryTesting extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot rob = new Robot(hardwareMap, telemetry, Robot.OPMODE_TYPE.DASHBOARD_TESTING);

        telemetry = rob.getTelemetry();

        Trajectory trajectory = new Trajectory(new Pose2D(0, 0,0));
        trajectory.add(new Pose2D(24, 24, 0));
        trajectory.add(new Point(24, 48));
        trajectory.add(new Point(45, 50));

        TelemetryPacket tp = DashboardDrawUtil.drawTrajectory(trajectory, new TelemetryPacket());
        FtcDashboard.getInstance().sendTelemetryPacket(tp);

        waitForStart();

        while (opModeIsActive() && !isStopRequested()) {
            telemetry.addData("Joe", Color.RED.format("joe"));
            FtcDashboard.getInstance().sendTelemetryPacket(tp);
            telemetry.update();
        }
    }
}
