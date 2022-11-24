package org.firstinspires.ftc.teamcode.hardware;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.opMode;
import static org.firstinspires.ftc.teamcode.hardware.subsystems.DriveTrain.lateralMultiplier;
import static org.firstinspires.ftc.teamcode.hardware.subsystems.DriveTrain.trackWidth;
import static org.firstinspires.ftc.teamcode.hardware.subsystems.DriveTrain.wheelBase;

import android.view.animation.LinearInterpolator;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.hardware.interfaces.Subsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystems.ConeManipulator;
import org.firstinspires.ftc.teamcode.hardware.subsystems.DriveTrain;
import org.firstinspires.ftc.teamcode.hardware.subsystems.IMU;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Intake;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Jimmy;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Lift;
import org.firstinspires.ftc.teamcode.hardware.subsystems.SleeveDetectionCamera;
import org.firstinspires.ftc.teamcode.hardware.subsystems.T265;
import org.firstinspires.ftc.teamcode.math.Pose2D;

import java.util.ArrayList;
import java.util.List;

public class Robot {
    public DriveTrain driveTrain;
    public ConeManipulator coneManipulator;
    public Intake intake;
    public Jimmy Jameson2Turnt;
    public Lift lift;
    public IMU imu;
    public T265 t265;
    public SleeveDetectionCamera sleeveDetectionCamera;

    public MultipleTelemetry telemetry;

    public HardwareMap hwMap;

    Subsystem[] subsystems = {};

    // Gamepad 1 & 2

    public Robot (HardwareMap hwMap, Telemetry o_telemetry) {
        this.hwMap = hwMap;
        driveTrain = new DriveTrain(this);
        intake = new Intake(this);          // DONE
        lift = new Lift(this);              // PENDING
        coneManipulator = new ConeManipulator(this);  // NOT STARTED
        //imu = new IMU(this);
        //t265 = new T265(hwMap);
        //Jameson2Turnt = new Jimmy(this);
        sleeveDetectionCamera = new SleeveDetectionCamera(this);


        subsystems = new Subsystem[] {driveTrain, intake, lift, sleeveDetectionCamera};
        //subsystems = new Subsystem[] {driveTrain, intake, lift, coneManipulator};

    }

    public double trackWidth() {
        return trackWidth;
    }

    public double wheelBase() {
        return wheelBase;
    }

    public double lateralMultiplier() {
        return lateralMultiplier;
    }

    public void resetEncoders() {
        driveTrain.resetEncoders();
    }

    public void setStartPosition(Pose2D start) {
        //localizer.setStartPosition(start);
        imu.setStartHeading(start.heading);
    }

    public List<Double> getWheelPositions() {
        List<Double> wheelPositions = new ArrayList<>();
        for (DcMotorEx motor : driveTrain.motors) {
            wheelPositions.add(convertTicksToInches(motor.getCurrentPosition()));
        }
        return wheelPositions;
    }

    public List<Double> getWheelVelocities() {
        List<Double> wheelVelocities = new ArrayList<>();
        for (DcMotorEx motor : driveTrain.motors) {
            wheelVelocities.add(convertTicksToInches(motor.getVelocity()));
        }
        return wheelVelocities;
    }

    public Double getExternalHeadingVelocity() {
        return imu.getExternalHeadingVelocity();
    }

    private double convertTicksToInches(double ticks) {
        return ticks * (2.0 * Math.PI * 3.77953 / 435); // Wheel radius / RPM
    }

    public void init() throws InterruptedException {
        for (Subsystem subsystem : subsystems){
            subsystem.init();
        }

        PhotonCore.enable();
    }
    public void update() throws InterruptedException {
        for (Subsystem subsystem : subsystems){
            subsystem.update();
        }
    }
}
