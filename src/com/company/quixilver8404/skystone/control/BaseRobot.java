package com.company.quixilver8404.skystone.control;

import com.company.quixilver8404.skystone.hardware.misc.Clock;
import com.company.simulator.MecanumKinematics;
import com.company.quixilver8404.skystone.util.measurement.Pose2D;

/**
 * Base robot class that facilitates basic navigation and control
 */
public abstract class BaseRobot {

//    private final HardwareLoopThread hardwareLoopThread;

    /**
     * Using this field outside of BaseRobot or its children should only be done for testing!
     */
    // Submodules
    public final PathFollowModule pathFollowModule;
    public final HeadingLockModule headingLockModule;
    public final NavModule navModule;
    public final DriveModule driveModule;
    public final MecanumKinematics kinematics;
    public final Clock clock;


    BaseRobot(Pose2D startPose, MecanumKinematics kinematics) {

//        hardwareLoopThread = new HardwareLoopThread(this);
        this.kinematics = kinematics;
        pathFollowModule = new PathFollowModule();
        headingLockModule = new HeadingLockModule(startPose.heading);
        navModule = new NavModule(startPose, kinematics);
        driveModule = new DriveModule();
        clock = new Clock();
    }

//    public void startHardwareLoop() {
//        if (!hardwareLoopThread.isAlive()) {
//            update();
//            hardwareLoopThread.start();
//        }
//    }
//
//    public void stopHardwareLoop() throws InterruptedException {
//        if (hardwareLoopThread.isAlive()) {
//            hardwareLoopThread.terminate();
//            hardwareLoopThread.join();
//        }
//    }

    public void update() {
        while (true) {
            pathFollowModule.update(this);
            headingLockModule.update(this);
            navModule.update();
            driveModule.update(this, kinematics);
        }
//        outputModule.update(this, hwCollection);
//        intakeModule.update(hwCollection);
//        foundationModule.update(this, hwCollection);
    }

//    public void waitForUpdate() {
//        if (!hardwareLoopThread.isAlive()) {
//            update();
//        } else {
//            int curUpdateCount = diagnosticModule.getUpdateCount();
//            while (opMode.opModeIsActive() && diagnosticModule.getUpdateCount() <= curUpdateCount) {
//                Thread.yield();
//            }
//        }
//    }
//
//    public void waitForFullUpdate() {
//        if (!hardwareLoopThread.isAlive()) {
//            update();
//        } else {
//            int curUpdateCount = diagnosticModule.getUpdateCount();
//            while (opMode.opModeIsActive() && diagnosticModule.getUpdateCount() <= curUpdateCount + 1) {
//                Thread.yield();
//            }
//        }
//    }
}
