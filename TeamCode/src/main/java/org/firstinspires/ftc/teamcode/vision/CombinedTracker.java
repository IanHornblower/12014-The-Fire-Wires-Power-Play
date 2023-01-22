package org.firstinspires.ftc.teamcode.vision;

import static org.opencv.core.Core.inRange;

import com.acmerobotics.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.openftc.easyopencv.OpenCvPipeline;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

@Config
public class CombinedTracker extends OpenCvPipeline {

    public enum TrackType {
        NONE,
        CONE,
        SLEEVE,
        POLE
    }

    public enum ParkingPosition {
        LEFT,
        CENTER,
        RIGHT
    }

    enum DETECT_COLOR {
        RED,
        BLUE,
        BOTH
    }

    public TrackType trackType = TrackType.NONE;

    public double CONTOUR_AREA = 100;
    private final Scalar CONTOUR_COLOR = new Scalar(255,0,255);
    private final Scalar HORIZON_COLOR = new Scalar(0,255,0);
    private final Scalar TEXT_COLOR = new Scalar(0, 0, 0);


    public DETECT_COLOR coneColor = DETECT_COLOR.BOTH;
    private volatile ParkingPosition position = ParkingPosition.LEFT;

    public double horizon = 110;

    private Rect redRect = new Rect();
    private Rect blueRect = new Rect();
    private Rect poleRect = new Rect();

    private final List<MatOfPoint> redContours = new ArrayList<>();
    private final List<MatOfPoint> blueContours = new ArrayList<>();

    private final ArrayList<MatOfPoint> contours = new ArrayList<>();
    // Cone mask scalars
    private final Scalar redLow = new Scalar(0, 161, 60);
    private final Scalar redHigh = new Scalar(200, 255, 255);
    private final Scalar blueLow = new Scalar(0, 80, 138);
    private final Scalar blueHigh = new Scalar(100, 255, 255);
    // Pole mask scalars
    public Scalar poleLower = new Scalar(95, 151, 66.6);
    public Scalar poleHigher = new Scalar(177.1, 191, 90);
    // Sleve mask scalars
    private final Scalar
            YELLOW  = new Scalar(255, 255, 0),
            CYAN    = new Scalar(0, 255, 255),
            MAGENTA = new Scalar(255, 0, 255);

    // Other Sleeve stuff

    // Width and height for the bounding box
    public static int REGION_WIDTH = 25;
    public static int REGION_HEIGHT = 35;

    // Anchor Point
    public static int anchorPointX = 182;
    public static int anchorPointY = 90;

    // TOPLEFT anchor point for the bounding box
    private static final Point SLEEVE_TOPLEFT_ANCHOR_POINT = new Point(anchorPointX, anchorPointY);

    // Anchor point definitions
    Point sleeve_pointA = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT.x,
            SLEEVE_TOPLEFT_ANCHOR_POINT.y);
    Point sleeve_pointB = new Point(
            SLEEVE_TOPLEFT_ANCHOR_POINT.x + REGION_WIDTH,
            SLEEVE_TOPLEFT_ANCHOR_POINT.y + REGION_HEIGHT);

    // Mat objects
    private final Mat maskRed = new Mat();
    private final Mat maskBlue = new Mat();
    private final Mat yCrCb = new Mat();
    private final Mat binaryMat = new Mat();

    private final Size kSize = new Size(5, 5);
    private final Mat kernel = Imgproc.getStructuringElement(Imgproc.MORPH_RECT, kSize);

    @Override
    public Mat processFrame(Mat input) {
        switch (trackType) {
            case CONE:
                return detectCone(input);
            case POLE:
                return detectPole(input);
            case SLEEVE:
                return detectSleeve(input);
            default:
                return input;
        }
    }

    //TODO: Convert to using only a single contour list
    private Mat detectCone(Mat input) {

        Imgproc.cvtColor(input, yCrCb, Imgproc.COLOR_RGB2YCrCb);
        Imgproc.erode(yCrCb, yCrCb, kernel);

        if (coneColor.equals(DETECT_COLOR.RED) || coneColor.equals(DETECT_COLOR.BOTH)) {
            inRange(yCrCb, redLow, redHigh, maskRed);

            redContours.clear();

            Imgproc.findContours(maskRed, redContours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            redContours.removeIf(c -> Imgproc.boundingRect(c).y + (Imgproc.boundingRect(c).height / 2.0) < horizon);
            Imgproc.drawContours(input, redContours, -1, CONTOUR_COLOR);

            if(!redContours.isEmpty()) {
                MatOfPoint biggestRedContour = Collections.max(redContours, Comparator.comparingDouble(t0 -> Imgproc.boundingRect(t0).width));
                if(Imgproc.contourArea(biggestRedContour) > CONTOUR_AREA) {
                    redRect = Imgproc.boundingRect(biggestRedContour);

                    Imgproc.rectangle(input, redRect, CONTOUR_COLOR, 2);
                    Imgproc.putText(input, "Red Cone", new Point(redRect.x, redRect.y < 10 ? (redRect.y+redRect.height+20) : (redRect.y - 8)), Imgproc.FONT_HERSHEY_SIMPLEX, 0.8, TEXT_COLOR, 1);
                }
            }

            maskRed.release();
        }

        if (coneColor.equals(DETECT_COLOR.BLUE) || coneColor.equals(DETECT_COLOR.BOTH)) {
            inRange(yCrCb, blueLow, blueHigh, maskBlue);

            blueContours.clear();

            Imgproc.findContours(maskBlue, blueContours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
            blueContours.removeIf(c -> Imgproc.boundingRect(c).y + (Imgproc.boundingRect(c).height / 2.0) < horizon);
            Imgproc.drawContours(input, blueContours, -1, CONTOUR_COLOR);

            if(!blueContours.isEmpty()) {
                MatOfPoint biggestBlueContour = Collections.max(blueContours, Comparator.comparingDouble(t0 -> Imgproc.boundingRect(t0).width));
                if(Imgproc.contourArea(biggestBlueContour) > CONTOUR_AREA) {
                    blueRect = Imgproc.boundingRect(biggestBlueContour);

                    Imgproc.rectangle(input, blueRect, CONTOUR_COLOR, 2);
                    Imgproc.putText(input, "Blue Cone", new Point(blueRect.x, blueRect.y < 10 ? (blueRect.y+blueRect.height+20) : (blueRect.y - 8)), Imgproc.FONT_HERSHEY_SIMPLEX, 0.8, TEXT_COLOR, 1);
                }
            }
            maskBlue.release();
        }

        Imgproc.line(input, new Point(0,horizon), new Point(640, horizon), HORIZON_COLOR);

        yCrCb.release();

        Imgproc.circle(input, VisionUtil.getCentroidOfRect(getBiggestCone()), 2, new Scalar(0, 255, 0), 2);

        return input;
    }

    private Mat detectPole(Mat input) {
        Imgproc.cvtColor(input, yCrCb, Imgproc.COLOR_RGB2YCrCb);
        Imgproc.erode(yCrCb, yCrCb, kernel);

        inRange(yCrCb, poleLower, poleHigher, binaryMat);

        Imgproc.findContours(binaryMat, contours, new Mat(), Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);
        contours.removeIf(c -> Imgproc.boundingRect(c).y + (Imgproc.boundingRect(c).height / 2.0) < horizon);
        Imgproc.drawContours(input, contours, -1, CONTOUR_COLOR);

        if(!contours.isEmpty()) {
            MatOfPoint biggestPole = Collections.max(contours, Comparator.comparingDouble(t0 -> Imgproc.boundingRect(t0).height));
            if(Imgproc.contourArea(biggestPole) > CONTOUR_AREA) {
                poleRect = Imgproc.boundingRect(biggestPole);

                Imgproc.rectangle(input, poleRect, CONTOUR_COLOR, 2);
                Imgproc.putText(input, "Pole", new Point(poleRect.x, poleRect.y < 10 ? (poleRect.y+poleRect.height+20) : (poleRect.y - 8)), Imgproc.FONT_HERSHEY_SIMPLEX, 0.8, TEXT_COLOR, 2);
                Imgproc.circle(input, VisionUtil.getCentroidOfRect(poleRect), 2, new Scalar(0, 255, 0), 2);
            }
        }

        Imgproc.line(input, new Point(0,horizon), new Point(640, horizon), HORIZON_COLOR);

        contours.clear();
        yCrCb.release();
        binaryMat.release();

        return input;
    }

    private Mat detectSleeve(Mat input) {
        Mat areaMat = input.submat(new Rect(sleeve_pointA, sleeve_pointB));
        Scalar sumColors = Core.sumElems(areaMat);

        double minColor = Math.min(sumColors.val[0], Math.min(sumColors.val[1], sumColors.val[2]));

        if (sumColors.val[0] == minColor) {
            position = ParkingPosition.CENTER;
            Imgproc.rectangle(
                    input,
                    sleeve_pointA,
                    sleeve_pointB,
                    CYAN,
                    2
            );
        } else if (sumColors.val[1] == minColor) {
            position = ParkingPosition.RIGHT;
            Imgproc.rectangle(
                    input,
                    sleeve_pointA,
                    sleeve_pointB,
                    MAGENTA,
                    2
            );
        } else {
            position = ParkingPosition.LEFT;
            Imgproc.rectangle(
                    input,
                    sleeve_pointA,
                    sleeve_pointB,
                    YELLOW,
                    2
            );
        }

        areaMat.release();
        return input;
    }

    public void setTrackType(TrackType trackType) {
        this.trackType = trackType;
    }


    public ParkingPosition getSleevePosition() {
        return position;
    }

    public Rect getBiggestCone() {
        return coneColor.equals(DETECT_COLOR.RED) ? redRect : blueRect;
    }

    public double getConeError() {
        return VisionUtil.getCentroidOfRect(getBiggestCone()).x - 320.0/2.0;
    }
}