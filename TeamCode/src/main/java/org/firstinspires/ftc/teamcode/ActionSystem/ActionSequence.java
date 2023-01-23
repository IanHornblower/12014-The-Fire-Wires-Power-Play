package org.firstinspires.ftc.teamcode.ActionSystem;

import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.ActionSystem.actions.CustomAction;
import org.firstinspires.ftc.teamcode.ActionSystem.actions.roadrunnerActions.FollowTrajectory;
import org.firstinspires.ftc.teamcode.ActionSystem.actions.Wait;
import org.firstinspires.ftc.teamcode.ActionSystem.actions.WaitFor;
import org.firstinspires.ftc.teamcode.hardware.Robot;

import java.util.ArrayList;
import java.util.function.BooleanSupplier;

public class ActionSequence {

    ArrayList<Action> actionSequence;

    Robot robot;

    public ActionSequence() {
        actionSequence = new ArrayList<>();
    }

    public ActionSequence(Robot robot) {
        actionSequence = new ArrayList<>();
        this.robot = robot;
    }

    public ActionSequence(ArrayList<Action> actionSequence) {
        this.actionSequence = actionSequence;
    }

    public ArrayList<Action> getActionList() {
        return actionSequence;
    }

    public void addAction(Action action) {
        actionSequence.add(action);
    }

    public void CustomAction(Runnable runnable) {
        actionSequence.add(new CustomAction(runnable));
    }

    public void WaitSeconds(double seconds) {
        actionSequence.add(new Wait(seconds));
    }

    public void WaitFor(BooleanSupplier supplier) {
        actionSequence.add(new WaitFor(supplier));
    }

    public void FollowTrajectory(Trajectory trajectory) throws InterruptedException {
        if(robot == null) throw new InterruptedException("Initialize Robot for Action Sequence");
        else actionSequence.add(new FollowTrajectory(robot, trajectory));
    }

}
