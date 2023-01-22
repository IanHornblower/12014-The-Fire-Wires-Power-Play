package org.firstinspires.ftc.teamcode.Roadrunner.trajectorysequence.sequencesegment;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.TrajectoryMarker;

import java.util.List;
import java.util.function.BooleanSupplier;

public class WaitForSegment extends SequenceSegment{
    public WaitForSegment(Pose2d pose, BooleanSupplier supplier, List<TrajectoryMarker> markers) {
        super(0, pose, pose, markers);
    }
}
