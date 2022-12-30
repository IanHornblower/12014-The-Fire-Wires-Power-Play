package org.firstinspires.ftc.teamcode.ActionSystem.actions.Depricatted;

import com.ThermalEquilibrium.homeostasis.Controllers.Feedback.BasicPID;

import org.firstinspires.ftc.teamcode.ActionSystem.Action;
import org.firstinspires.ftc.teamcode.hardware.DriveConstraints;
import org.firstinspires.ftc.teamcode.hardware.subsystems.DriveTrain;

public class ForwardBackwardControl extends Action {

    BasicPID fwController = new BasicPID(DriveConstraints.forwardBackwardPID);

    DriveTrain dt;
    double totalTicks;
    int ticks;

    public ForwardBackwardControl(DriveTrain dt, int ticks) {
        this.dt = dt;
        this.ticks = ticks;
    }

    @Override
    public void startAction() {
    }

    @Override
    public void runAction() throws InterruptedException {
        totalTicks = dt.motors[0].getCurrentPosition();

        fwController.calculate(ticks, totalTicks);

        dt.setMotorPowers(fwController.calculate(ticks, totalTicks), fwController.calculate(ticks, totalTicks), fwController.calculate(ticks, totalTicks), fwController.calculate(ticks, totalTicks));

        isComplete = Math.abs(totalTicks) > Math.abs(ticks);
    }

    @Override
    public void stopAction() {
        dt.stopDriveTrain();
    }
}
