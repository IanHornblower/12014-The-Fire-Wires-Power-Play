package org.firstinspires.ftc.teamcode.hardware.subsystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.checkerframework.checker.units.qual.A;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.stream.CameraStreamServer;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.interfaces.Subsystem;
import org.firstinspires.ftc.teamcode.util.Color;
import org.firstinspires.ftc.teamcode.vision.AprilTagDetectionPipeline;
import org.firstinspires.ftc.teamcode.vision.CombinedTracker;
import org.openftc.apriltag.AprilTagDetection;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;

import java.util.ArrayList;

@Config
public class RearCamera implements Subsystem {

    public CombinedTracker combinedTracker;
    public OpenCvCamera camera;

    // Name of the Webcam to be set in the config
    String webcamName = "BackCamera";

    int resWidth = 1280;
    int resHeight = 720;

    public static double fx = 578.272;
    public static double fy = 578.272;
    public static double cx = 402.145;
    public static double cy = 221.506;

    // UNITS ARE METERS
    public static double tagsize = 0.166;

    int LEFT = 9;
    int MIDDLE = 16;
    int RIGHT = 11;

    public enum State {
        LEFT,
        RIGHT,
        MIDDLE,
        NONE
    }

    public State state = State.NONE;

    AprilTagDetection tagOfInterest = null;

    OpenCvCameraRotation cameraRotation = OpenCvCameraRotation.UPSIDE_DOWN;

    public CombinedTracker.ParkingPosition position = CombinedTracker.ParkingPosition.LEFT;
    public AprilTagDetectionPipeline aprilTagDetectionPipeline;

    public RearCamera(Robot robot, HardwareMap hwMap) {
        int cameraMonitorViewId = hwMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hwMap.appContext.getPackageName());
        camera = OpenCvCameraFactory.getInstance().createWebcam(hwMap.get(WebcamName.class, webcamName), cameraMonitorViewId);
       // combinedTracker = new CombinedTracker();
       // combinedTracker.setTrackType(CombinedTracker.TrackType.SLEEVE);
        aprilTagDetectionPipeline = new AprilTagDetectionPipeline(tagsize, fx, fy, cx, cy);
        camera.setPipeline(aprilTagDetectionPipeline);
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

   // public void reInit() {
   //  //   combinedTracker.reInitSleevePoints();
   // }

    public void setCameraRotation(OpenCvCameraRotation rotation) {
        cameraRotation = rotation;
    }

   // public double getObjectError() {
   //     return combinedTracker.getConeError();
   // }

    public State getSleeveLocation() {
        return state;
    }

    public String getTelemetry() {
        switch (getSleeveLocation()) {
            case LEFT:
                return Color.NO_COLOR.format("Sleeve Detection: ") + Color.YELLOW.format("LEFT");
            case MIDDLE:
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
        ArrayList<AprilTagDetection> currentDetections = aprilTagDetectionPipeline.getLatestDetections();
        if(currentDetections.size() != 0) {
            boolean tagFound = false;

            for(AprilTagDetection tag : currentDetections) {
                if(tag.id == LEFT || tag.id == MIDDLE || tag.id == RIGHT) {
                    tagOfInterest = tag;
                    tagFound = true;
                    break;
                }
            }

            if(tagFound) {
                //telemetry.addLine("Tag of interest is in sight!\n\nLocation data:");
                //tagToTelemetry(tagOfInterest);
            }
            else {
               // telemetry.addLine("Don't see tag of interest :(");

                if(tagOfInterest == null) {
                 //   telemetry.addLine("(The tag has never been seen)");
                }
                else {
                    //telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
                    //tagToTelemetry(tagOfInterest);
                }
            }

        }
        else {
            //telemetry.addLine("Don't see tag of interest :(");

            if(tagOfInterest == null) {
            //    telemetry.addLine("(The tag has never been seen)");
            }
            else {
             //   telemetry.addLine("\nBut we HAVE seen the tag before; last seen at:");
            }

        }

        /*
         * The START command just came in: now work off the latest snapshot acquired
         * during the init loop.
         */

        /* Update the telemetry */
        if(tagOfInterest != null)
        {
           // telemetry.addLine("Tag snapshot:\n");
           // tagToTelemetry(tagOfInterest);
           // telemetry.update();
        }
        else
        {
          // telemetry.addLine("No tag snapshot available, it was never sighted during the init loop :(");
          // telemetry.update();
        }

        /* Actually do something useful */
        if(tagOfInterest == null){
            state = State.NONE;
        }else if(tagOfInterest.id == LEFT){
            state = State.LEFT;
        }else if(tagOfInterest.id == MIDDLE){
            state = State.MIDDLE;
        }else{
            state = State.RIGHT;
        }
    }




    }
