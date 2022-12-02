package org.firstinspires.ftc.teamcode.hardware.subsystems

import com.acmerobotics.roadrunner.geometry.Pose2d
import com.acmerobotics.roadrunner.kinematics.Kinematics
import com.acmerobotics.roadrunner.kinematics.MecanumKinematics
import com.acmerobotics.roadrunner.localization.Localizer
import com.acmerobotics.roadrunner.util.Angle
import org.firstinspires.ftc.teamcode.hardware.Robot

class Localizer @JvmOverloads constructor(
        private val drive: Robot,
        private val useExternalHeading: Boolean = true
) : Localizer {
    private var _poseEstimate = Pose2d()
    override var poseEstimate: Pose2d
        get() = _poseEstimate
        set(value) {
            lastWheelPositions = emptyList()
            lastExtHeading = Double.NaN
            _poseEstimate = value
        }
    override var poseVelocity: Pose2d? = null
        private set
    private var lastWheelPositions = emptyList<Double>()
    private var lastExtHeading = Double.NaN

    override fun update() {
        val wheelPositions = drive.wheelPositions
        val extHeading = if (useExternalHeading) drive.imu.headingInRadians else Double.NaN
        if (lastWheelPositions.isNotEmpty()) {
            val wheelDeltas = wheelPositions
                    .zip(lastWheelPositions)
                    .map { it.first - it.second }
            val robotPoseDelta = MecanumKinematics.wheelToRobotVelocities(
                    wheelDeltas,
                    drive.trackWidth(),
                    drive.wheelBase(),
                    drive.lateralMultiplier()
            )
            val finalHeadingDelta = if (useExternalHeading) {
                Angle.normDelta(extHeading - lastExtHeading)
            } else {
                robotPoseDelta.heading
            }
            _poseEstimate = Kinematics.relativeOdometryUpdate(
                    _poseEstimate,
                    Pose2d(robotPoseDelta.vec(), finalHeadingDelta)
            )
        }

        val wheelVelocities = drive.wheelVelocities
        val extHeadingVel = drive.externalHeadingVelocity
        if (wheelVelocities != null) {
            poseVelocity = MecanumKinematics.wheelToRobotVelocities(
                    wheelVelocities,
                    drive.trackWidth(),
                    drive.wheelBase(),
                    drive.lateralMultiplier()
            )
            if (useExternalHeading && extHeadingVel != null) {
                poseVelocity = Pose2d(poseVelocity!!.vec(), extHeadingVel)
            }
        }

        lastWheelPositions = wheelPositions
        lastExtHeading = extHeading
    }
}