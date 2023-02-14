package org.firstinspires.ftc.teamcode.ActionSystem;

import com.acmerobotics.roadrunner.trajectory.Trajectory;

import org.firstinspires.ftc.teamcode.ActionSystem.actions.CustomAction;
import org.firstinspires.ftc.teamcode.ActionSystem.actions.DropAndReturn;
import org.firstinspires.ftc.teamcode.ActionSystem.actions.LiftSetPosition;
import org.firstinspires.ftc.teamcode.ActionSystem.actions.V4BSetPosition;
import org.firstinspires.ftc.teamcode.ActionSystem.actions.roadrunnerActions.FollowTrajectory;
import org.firstinspires.ftc.teamcode.ActionSystem.actions.Wait;
import org.firstinspires.ftc.teamcode.ActionSystem.actions.WaitFor;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.ConeManipulator;
import org.firstinspires.ftc.teamcode.hardware.subsystems.Lift;

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

    public void customAction(Runnable runnable) {
        actionSequence.add(new CustomAction(runnable));
    }

    public void waitSeconds(double seconds) {
        actionSequence.add(new Wait(seconds));
    }

    public void waitFor(BooleanSupplier supplier) {
        actionSequence.add(new WaitFor(supplier));
    }

    public void followTrajectory(Trajectory trajectory) throws InterruptedException {
        if(robot == null) throw new InterruptedException("Initialize Robot for Action Sequence");
        else actionSequence.add(new FollowTrajectory(robot, trajectory));
    }

    public void liftTo(double position) throws InterruptedException {
        if(robot == null) throw new InterruptedException("Initialize Robot for Action Sequence");
        else actionSequence.add(new LiftSetPosition(robot, position));
    }

    public void liftTo(Lift.LIFT level) throws InterruptedException {
        if(robot == null) throw new InterruptedException("Initialize Robot for Action Sequence");
        else actionSequence.add(new LiftSetPosition(robot, level.getTicks()));
    }

    public void liftTo(double position, boolean instant) throws InterruptedException {
        if(robot == null) throw new InterruptedException("Initialize Robot for Action Sequence");
        else actionSequence.add(new LiftSetPosition(robot, position, instant));
    }

    public void liftTo(Lift.LIFT level, boolean instant) throws InterruptedException {
        if(robot == null) throw new InterruptedException("Initialize Robot for Action Sequence");
        else actionSequence.add(new LiftSetPosition(robot, level.getTicks(), instant));
    }

    public void returnLift() throws InterruptedException{
        if(robot == null) throw new InterruptedException("Initialize Robot for Action Sequence");
        else actionSequence.add(new LiftSetPosition(robot, Lift.LIFT.RETURN.getTicks()));
    }

    public void returnLift(boolean instant) throws InterruptedException{
        if(robot == null) throw new InterruptedException("Initialize Robot for Action Sequence");
        else actionSequence.add(new LiftSetPosition(robot, Lift.LIFT.RETURN.getTicks(), instant));
    }

    public void fourbarTo(double position) throws InterruptedException{
        if(robot == null) throw new InterruptedException("Initialize Robot for Action Sequence");
        else actionSequence.add(new V4BSetPosition(robot, position));
    }

    public void fourbarTo(double position, double tolerance) throws InterruptedException {
        if(robot == null) throw new InterruptedException("Initialize Robot for Action Sequence");
        else actionSequence.add(new V4BSetPosition(robot, position, tolerance));
    }

    public void fourbarTo(ConeManipulator.V4BPreset position) throws InterruptedException {
        if(robot == null) throw new InterruptedException("Initialize Robot for Action Sequence");
        else actionSequence.add(new V4BSetPosition(robot, position));
    }

    public void fourbarTo(ConeManipulator.V4BPreset position, double tolerance) throws InterruptedException {
        if(robot == null) throw new InterruptedException("Initialize Robot for Action Sequence");
        else actionSequence.add(new V4BSetPosition(robot, position, tolerance));
    }

    public void fourbarTo(double position, boolean instant) throws InterruptedException{
        if(robot == null) throw new InterruptedException("Initialize Robot for Action Sequence");
        else actionSequence.add(new V4BSetPosition(robot, position, instant));
    }

    public void fourbarTo(double position, double tolerance, boolean instant) throws InterruptedException {
        if(robot == null) throw new InterruptedException("Initialize Robot for Action Sequence");
        else actionSequence.add(new V4BSetPosition(robot, position, tolerance, instant));
    }

    public void fourbarTo(ConeManipulator.V4BPreset position, boolean instant) throws InterruptedException {
        if(robot == null) throw new InterruptedException("Initialize Robot for Action Sequence");
        else actionSequence.add(new V4BSetPosition(robot, position, instant));
    }

    public void fourbarTo(ConeManipulator.V4BPreset position, double tolerance, boolean instant) throws InterruptedException {
        if(robot == null) throw new InterruptedException("Initialize Robot for Action Sequence");
        else actionSequence.add(new V4BSetPosition(robot, position, tolerance, instant));
    }

    public void openClaw() throws InterruptedException {
        if(robot == null) throw new InterruptedException("Initialize Robot for Action Sequence");
        else actionSequence.add(new CustomAction(()-> robot.coneManipulator.open()));
    }

    public void closeClaw() throws InterruptedException {
        if(robot == null) throw new InterruptedException("Initialize Robot for Action Sequence");
        else actionSequence.add(new CustomAction(()-> robot.coneManipulator.close()));
    }

    public void transferClaw() throws InterruptedException {
        if(robot == null) throw new InterruptedException("Initialize Robot for Action Sequence");
        else actionSequence.add(new CustomAction(()-> robot.coneManipulator.middle()));
    }

    public void retractOdom() throws InterruptedException {
        if(robot == null) throw new InterruptedException("Initialize Robot for Action Sequence");
        else actionSequence.add(new CustomAction(()-> robot.retract()));
    }

    public void releasetOdom() throws InterruptedException {
        if(robot == null) throw new InterruptedException("Initialize Robot for Action Sequence");
        else actionSequence.add(new CustomAction(()-> robot.release()));
    }

    public void intakeOut() throws InterruptedException {
        if(robot == null) throw new InterruptedException("Initialize Robot for Action Sequence");
        else actionSequence.add(new CustomAction(()-> robot.intake.setPower(1)));
    }

    public void intakeIn() throws InterruptedException {
        if(robot == null) throw new InterruptedException("Initialize Robot for Action Sequence");
        else actionSequence.add(new CustomAction(()-> robot.intake.setPower(-1)));
    }

    public void intakeOff() throws InterruptedException {
        if(robot == null) throw new InterruptedException("Initialize Robot for Action Sequence");
        else actionSequence.add(new CustomAction(()-> robot.intake.setPower(0.0)));
    }

    public void dropAndReturn() throws InterruptedException {
        if(robot == null) throw new InterruptedException("Initialize Robot for Action Sequence");
        else actionSequence.add(new DropAndReturn(robot));
    }

    public void dropAndReturn(boolean quickDrop) throws InterruptedException {
        if(robot == null) throw new InterruptedException("Initialize Robot for Action Sequence");
        else actionSequence.add(new DropAndReturn(robot, quickDrop));
    }

    public void dropAndReturn(double level) throws InterruptedException {
        if(robot == null) throw new InterruptedException("Initialize Robot for Action Sequence");
        else actionSequence.add(new DropAndReturn(robot, level));
    }

    public void dropAndReturn(double level, boolean quickDrop) throws InterruptedException {
        if(robot == null) throw new InterruptedException("Initialize Robot for Action Sequence");
        else actionSequence.add(new DropAndReturn(robot, level, quickDrop));
    }
}
