package org.firstinspires.ftc.teamcode.hardware;

import com.ThermalEquilibrium.homeostasis.Parameters.PIDCoefficients;
import com.acmerobotics.dashboard.config.Config;
@Config
public class DriveConstraints {
    // Drivetrain
    public static double wheelBase = 9.8;
    public static double dt_trackWidth = 15;
    public static double maxVelocity = 28.05;
    public static double maxAngularVelocity = 107.121;
    public static double encoderResolution = 384.5;
    public static double lateralMultiplier = 1;
    public static double VX_WEIGHT = 1;
    public static double VY_WEIGHT = 1;
    public static double TURN_WEIGHT = 1;

    public static PIDCoefficients xPID = new PIDCoefficients(0.02, 0 ,0);
    public static PIDCoefficients yPID = new PIDCoefficients(0.02, 0 ,0);
    public static PIDCoefficients headingPID = new PIDCoefficients(0.08, 0 ,0);
    public static PIDCoefficients forwardBackwardPID = new PIDCoefficients(0.002, 0, 0);


    public static PIDCoefficients inplace_headingPID = new PIDCoefficients(0.55, 0 ,0.1);

    public static PIDCoefficients trajectoryPID = new PIDCoefficients(0.07, 0 ,0);

    // Ex Stuff
    public static double[] EncoderDriveHeadingPID = {0, 0, 0};

    // Dead wheels [Prolly won't exist]
    public static double trackWidth = 8.6;
    public static double ticksPerRevolution = 8192;
    public static double wheelRadius = 0.688975;
    public static double lateralOffset = -0.41;
}
