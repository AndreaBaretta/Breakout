package com.company.quixilver8404.skystone.control;

/**
 * Calls robot.update() repeatedly on a separate thread until stopped
 */
public class HardwareLoopThread extends Thread { // TODO make useful

    private volatile boolean terminateRequested = false;
    private final BaseRobot robot;

    public HardwareLoopThread(BaseRobot robot) {
        this.robot = robot;
    }

    @Override
    public void run() {
        terminateRequested = false;
        while (!terminateRequested) {
            robot.update();
        }
    }

    public void terminate() {
        terminateRequested = true;
    }

    public boolean isTerminateRequested() {
        return terminateRequested;
    }
}
