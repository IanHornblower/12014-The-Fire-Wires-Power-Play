package org.firstinspires.ftc.teamcode.ActionSystem.actions;

import org.firstinspires.ftc.teamcode.ActionSystem.Action;
import org.firstinspires.ftc.teamcode.hardware.Robot;
import org.firstinspires.ftc.teamcode.hardware.subsystems.ConeManipulator;
import org.firstinspires.ftc.teamcode.util.Timer;

public class DropAndReturn extends Action {

    enum STATE {
        START,
        DROP,
        OPEN_AFTER_DROP,
        RETURN_TO_PRIME,
        CLOSE_AFTER_PRIME,
        RETURN_LIFT,
        OPEN_AFTER_RETURN,
        DONE
    }

    STATE state;

    boolean quickDrop = false;
    double level = 0;
    Robot robot;
    Timer timer = new Timer();

    public DropAndReturn(Robot robot) {
        this.robot = robot;
    }

    public DropAndReturn(Robot robot, boolean quickDrop) {
        this.robot = robot;
        this.quickDrop = quickDrop;
    }

    public DropAndReturn(Robot robot, double level) {
        this.robot = robot;
        this.level = level;
    }

    public DropAndReturn(Robot robot, double level, boolean quickDrop) {
        this.robot = robot;
        this.quickDrop = quickDrop;
        this.level = level;
    }

    @Override
    public void startAction() {
        state = STATE.START;
    }

    @Override
    public void runAction() throws InterruptedException {
        switch(state) {
            case START:
                timer.start();
                if(quickDrop) state = STATE.OPEN_AFTER_DROP;
                else state = STATE.DROP;
                break;
            case DROP:
                robot.coneManipulator.setPosition(ConeManipulator.V4BPreset.CORRECT_RIGHT);

                if(Math.abs(robot.coneManipulator.error) < 50) {
                    state = STATE.OPEN_AFTER_DROP;
                    timer.reset();
                }
                break;
            case OPEN_AFTER_DROP:
                robot.coneManipulator.open();

                if(timer.currentSeconds() > 0.15) {
                    timer.reset();
                    state = STATE.RETURN_TO_PRIME;
                }
                break;
            case RETURN_TO_PRIME:
                robot.coneManipulator.setPosition(ConeManipulator.V4BPreset.PRIME);

                state = STATE.CLOSE_AFTER_PRIME;
                break;
            case CLOSE_AFTER_PRIME:
                robot.coneManipulator.close();

                if(timer.currentSeconds() > 0.2 && !quickDrop) {
                    timer.reset();
                    state = STATE.RETURN_LIFT;
                }
                else if(timer.currentSeconds() > 0.4 && quickDrop) {
                    timer.reset();
                    state = STATE.RETURN_LIFT;
                }
                break;
            case RETURN_LIFT:
                robot.lift.setPosition(level);

                if(Math.abs(error) < 30) state = STATE.OPEN_AFTER_RETURN;
                break;
            case OPEN_AFTER_RETURN:
                robot.coneManipulator.open();
                state = STATE.DONE;
                break;
            case DONE:
                isComplete = true;
                break;
        }
    }

    @Override
    public void stopAction() {

    }
}
