package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamServer;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.interfaces.Subsystem;
import org.firstinspires.ftc.teamcode.util.Color;
import org.firstinspires.ftc.teamcode.vision.CombinedTracker;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class RearCamera implements Subsystem {

    public CombinedTracker combinedTracker;
    public OpenCvCamera camera;

    // Name of the Webcam to be set in the config
    String webcamName = "BackCamera";

    int resWidth = 320;
    int resHeight = 240;
    OpenCvCameraRotation cameraRotation = OpenCvCameraRotation.UPSIDE_DOWN;
    public CombinedTracker.ParkingPosition position = CombinedTracker.ParkingPosition.LEFT;

    public RearCamera(Robot robot, HardwareMap hwMap) {
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        combinedTracker = new CombinedTracker();
        combinedTracker.setTrackType(CombinedTracker.TrackType.SLEEVE);
        camera.setPipeline(combinedTracker);
    }

    public OpenCvCamera getCamera() {
        return camera;
    }

    public void setCameraResolution(int width, int height) {
        resWidth = width;
        resHeight = height;
    }

    public void disableCamera() {
        camera.stopRecordingPipeline();
        camera.stopStreaming();
    }

    public void setCameraRotation(OpenCvCameraRotation rotation) {
        cameraRotation = rotation;
    }

    public double getObjectError() {
        return combinedTracker.getConeError();
    }

    public CombinedTracker.ParkingPosition getSleeveLocation() {
        return combinedTracker.getSleevePosition();
    }

    public String getTelemetry() {
        switch (getSleeveLocation()) {
            case LEFT:
                return Color.NO_COLOR.format("Sleeve Detection: ") + Color.YELLOW.format("LEFT");
            case CENTER:
                return Color.NO_COLOR.format("Sleeve Detection: ") + Color.CYAN.format("CENTER");
            case RIGHT:
                return Color.NO_COLOR.format("Sleeve Detection: ") + Color.MAGENTA.format("RIGHT");
            default:
                return Color.NO_COLOR.format("Sleeve Detection: ") + Color.RED.format("ERROR NO CONE FOUND");
        }
    }

    @Override
    public void init() throws InterruptedException {
        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(resWidth,resHeight, cameraRotation);
            }

            @Override
            public void onError(int errorCode) {}
        });

        FtcDashboard.getInstance().startCameraStream(camera, 0);
        CameraStreamServer.getInstance().setSource(camera);
    }

    @Override
    public void update() throws InterruptedException {
    }
}
