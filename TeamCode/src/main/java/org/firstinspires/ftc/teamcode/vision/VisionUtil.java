package org.firstinspires.ftc.teamcode.vision;

import org.opencv.core.Point;
import org.opencv.core.Rect;

public class VisionUtil {
    public static Point getCentroidOfRect(Rect rect) {
        double x = (rect.tl().x + rect.br().x) / 2.0;
        double y = (rect.tl().y + rect.br().y) / 2.0;

        return new Point(x, y);
    }
}
