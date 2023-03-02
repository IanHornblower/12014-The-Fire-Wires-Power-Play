package org.firstinspires.ftc.teamcode.hardware;

import android.graphics.Color;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.outoftheboxrobotics.photoncore.PhotonCore;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Roadrunner.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.hardware.interfaces.Subsystem;
import org.firstinspires.ftc.teamcode.hardware.subsystems.ConeManipulator;
import org.firstinspires.ftc.teamcode.hardware.subsystems.IMU;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Intake;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Jimmy;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Lift;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Mogus;
import org.firstinspires.ftc.teamcode.hardware.subsystems.RearCamera;
import org.firstinspires.ftc.teamcode.hardware.subsystems.StandardTrackingWheelLocalizer;

import java.util.function.BooleanSupplier;

public class Robot {
    public SampleMecanumDrive driveTrain;
    public ConeManipulator coneManipulator;
    public Intake intake;
    public Jimmy Jameson2Turnt;
    public Lift lift;
    public IMU imu;
    public RearCamera rearCamera;
    public StandardTrackingWheelLocalizer localizer;
    public Mogus mogus;

    public final FtcDashboard FtcDashboardInstance = FtcDashboard.getInstance();

    Telemetry telemetry;

    public HardwareMap hwMap;

    public BooleanSupplier tempSideFlip = ()-> false;

    public Subsystem[] subsystems = {};

    public enum OPMODE_TYPE {
        AUTO(0),
        TELEOP(1),
        DASHBOARD_TESTING(2);

        int v;

        OPMODE_TYPE(int v) {
            this.v = v;
        }

        public int getValue() {
            return v;
        }
    }

    public Servo leftRetract, rightRetract, latteralRetract;
    public int coneStack = 5;

    public Robot (HardwareMap hwMap, Telemetry telemetry, OPMODE_TYPE type) {
        this.hwMap = hwMap;

        telemetry.setDisplayFormat(Telemetry.DisplayFormat.HTML);
        this.telemetry = new MultipleTelemetry(telemetry, FtcDashboardInstance.getTelemetry());

        switch (type.getValue()) {

            case 0: // Auto
                driveTrain = new SampleMecanumDrive(this);
                intake = new Intake(this);
                lift = new Lift(this);
                coneManipulator = new ConeManipulator(this);
                mogus = new Mogus();
               rearCamera = new RearCamera(this, hwMap);

                subsystems = new Subsystem[] {driveTrain, intake, lift, coneManipulator, mogus};
               subsystems = new Subsystem[] {driveTrain, intake, lift, coneManipulator, rearCamera, mogus};
                break;
            case 1: // TeleOp
                driveTrain = new SampleMecanumDrive(this);
                intake = new Intake(this);
                lift = new Lift(this);
                mogus = new Mogus();
                coneManipulator = new ConeManipulator(this);

                subsystems = new Subsystem[] {driveTrain, intake, lift, coneManipulator, mogus};
                break;
            case 2: // Testing
                mogus = new Mogus();
                subsystems = new Subsystem[] {mogus};
                break;

        }

    }

    public Telemetry getTelemetry() {
        return telemetry;
    }

    public void init() throws InterruptedException {
        for (Subsystem subsystem : subsystems){
            subsystem.init();
        }

        leftRetract = hwMap.get(Servo.class, "leftRe");
        rightRetract = hwMap.get(Servo.class, "rightRe");
        latteralRetract = hwMap.get(Servo.class, "latRe");

        //PhotonCore.enable();
    }

    public void retract() {
        leftRetract.setPosition(0);//
        rightRetract.setPosition(0);
        latteralRetract.setPosition(1);
    }

    public void release() {
        leftRetract.setPosition(1);//
        rightRetract.setPosition(1);
        latteralRetract.setPosition(0);
    }

    public void update() throws InterruptedException {
        for (Subsystem subsystem : subsystems){
            subsystem.update();
        }
    }


}
