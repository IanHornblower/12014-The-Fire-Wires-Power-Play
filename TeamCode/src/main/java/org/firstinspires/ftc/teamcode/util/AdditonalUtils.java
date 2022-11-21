package org.firstinspires.ftc.teamcode.util;

import com.ThermalEquilibrium.homeostasis.Utils.Vector;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;

import org.firstinspires.ftc.teamcode.math.Pose2D;

public class AdditonalUtils {

	/**
	 * rotate a 2 item vector by an angle radians
	 * @param v two item vector, x:0, y:1
	 * @param angleRadians angle in radians
	 */
	public static void rotate(Vector v, double angleRadians) {
		double cosA = Math.cos(angleRadians);
		double sinA = Math.sin(angleRadians);

		double x = v.get(0) * cosA - v.get(1) * sinA;
		double y = v.get(0) * sinA - v.get(1) * cosA;

		v.set(x,0);
		v.set(y,1);
	}

	public static double calculateDistance(Vector v1, Vector v2) {
		double x1 = v1.get(0);
		double x2 = v2.get(0);
		double y1 = v1.get(1);
		double y2 = v2.get(1);
		return Math.sqrt(Math.pow(x2 - x1, 2) + Math.pow(y2 - y1, 2));
	}
	
	public static double atan2(Vector v) {
		return Math.atan2(v.get(1), v.get(0));
	}

	public static Pose2d ConvertPose2DtoPose2d(Pose2D pose) {
		return new Pose2d(pose.x, pose.y, new Rotation2d(pose.heading));
	}

	public static Pose2D ConvertPose2dtoPose2D(Pose2d pose) {
		return new Pose2D(pose.getX(), pose.getY(), pose.getHeading());
	}

}
