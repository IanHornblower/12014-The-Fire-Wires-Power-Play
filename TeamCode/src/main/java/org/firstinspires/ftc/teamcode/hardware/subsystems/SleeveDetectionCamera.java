package org.firstinspires.ftc.teamcode.hardware.subsystems;

import org.firstinspires.ftc.robotcore.external.hardware.camera.Camera;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.interfaces.Subsystem;
import org.firstinspires.ftc.teamcode.util.Color;
import org.firstinspires.ftc.teamcode.vision.SleeveDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

public class SleeveDetectionCamera implements Subsystem {

    SleeveDetection sleeveDetection;
    OpenCvCamera camera;

    // Name of the Webcam to be set in the config
    String webcamName = "BackCamera";

    int resWidth = 320;
    int resHeight = 240;
    OpenCvCameraRotation cameraRotation = OpenCvCameraRotation.UPRIGHT;
    public SleeveDetection.ParkingPosition position = SleeveDetection.ParkingPosition.LEFT;

    public SleeveDetectionCamera(Robot robot) {
        int cameraMonitorViewId = robot.hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", robot.hwMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(robot.hwMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
        sleeveDetection = new SleeveDetection();
        camera.setPipeline(sleeveDetection);
    }

    public void setCameraResolution(int width, int height) {
        resWidth = width;
        resHeight = height;
    }

    public void setCameraRotation(OpenCvCameraRotation rotation) {
        cameraRotation = rotation;
    }

    public SleeveDetection.ParkingPosition getPosition() {
        return position;
    }

    public String getTelemetry() {
        switch (position) {
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
    }

    @Override
    public void update() throws InterruptedException {
        position = sleeveDetection.getPosition();
    }
}
