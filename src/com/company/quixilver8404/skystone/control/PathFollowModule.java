package com.company.quixilver8404.skystone.control;

import com.company.quixilver8404.skystone.util.measurement.Angle;
import com.company.quixilver8404.skystone.util.measurement.Distance;
import com.company.quixilver8404.skystone.util.measurement.Pose2D;

public class PathFollowModule {

    // every unspecified unit for distance will be in inches
    public static final double LOOKAHEAD_MIN_INCHES = 1.5; // TODO tune
    public static final double LOOKAHEAD_MAX_INCHES = 4.0; // TODO tune
    public static final double STOP_THRESHOLD_INCHES = 1.5;

    public static final double MIN_POWER = 0.25; // TODO tune
    public static final double FULL_ACCELERATION_DIST_INCHES = 8.0; // TODO tune
    public static final double FULL_DECELERATION_DIST_INCHES = 20.0; // TODO tune
    public static final double MAX_ACCELERATION_POWER_PER_INCH = 1.0 / (2 * FULL_ACCELERATION_DIST_INCHES);
    public static final double MAX_DECELERATION_POWER_PER_INCH = 1.0 / (2 * FULL_DECELERATION_DIST_INCHES);
    public static final double POWER_PER_INCH_RADIUS = 1.0 / 25.0; // TODO tune

    // windows relative to the last lookahead distance
    public static final double LOOKAHEAD_WINDOW_AHEAD_INCHES = 4.0; // TODO tune
    public static final double LOOKAHEAD_WINDOW_BEHIND_INCHES = 1.0; // TODO tune
    public static final double LOOKAHEAD_WINDOW_SIZE_INCHES = LOOKAHEAD_WINDOW_BEHIND_INCHES + LOOKAHEAD_WINDOW_AHEAD_INCHES;

    private boolean isBusy;
    private PurePursuitPath curPath;
    private double curLookaheadWindowStartInches;
    private double lastPower;

    private double lastClosestT;
    private double lastLookaheadT;

    private ActionSet curActionSet;

    public PathFollowModule() {
        reset();
    }

    public synchronized void update(BaseRobot baseRobot) {
        if (isBusy) {
            Pose2D pose = baseRobot.navModule.getPose();
            double x = pose.x.getValue(Distance.Unit.INCHES);
            double y = pose.y.getValue(Distance.Unit.INCHES);

            double lookaheadDist = LOOKAHEAD_MIN_INCHES + lastPower * (LOOKAHEAD_MAX_INCHES - LOOKAHEAD_MIN_INCHES);

            if (curPath.length() - (curLookaheadWindowStartInches + lookaheadDist) < STOP_THRESHOLD_INCHES && curPath.distFromEnd(x, y) < STOP_THRESHOLD_INCHES) {
                // reached the end
                baseRobot.driveModule.setExtrinsicTargetPower(0, Angle.ZERO);
                double headingP = curPath.heading(curPath.upper());
                baseRobot.headingLockModule.setTargetHeading(new Angle(headingP, Angle.Unit.RADIANS));
                curActionSet.runRemaining(baseRobot);
                reset(); // sets isBusy to false
            } else {

                double lookaheadT = curPath.lookahead(x, y, lookaheadDist,
                        curPath.atDist(curLookaheadWindowStartInches), curPath.atDist(curLookaheadWindowStartInches + LOOKAHEAD_WINDOW_SIZE_INCHES));
                double closestT = curPath.closest(x, y,
                        curPath.atDist(curLookaheadWindowStartInches), curPath.atDist(curLookaheadWindowStartInches + LOOKAHEAD_WINDOW_SIZE_INCHES));
                lastLookaheadT = lookaheadT;
                lastClosestT = closestT;

                double newLookaheadWindowStartInches = curPath.distAlongPaths(lookaheadT) - LOOKAHEAD_WINDOW_BEHIND_INCHES;
                if (newLookaheadWindowStartInches > curLookaheadWindowStartInches) {
                    curLookaheadWindowStartInches = newLookaheadWindowStartInches;
                }

                double xP = curPath.x(lookaheadT);
                double yP = curPath.y(lookaheadT);
                double headingP = curPath.heading(lookaheadT);
                double powerP = curPath.power(lookaheadT);
                if (powerP < MIN_POWER) {
                    powerP = MIN_POWER;
                }
                lastPower = powerP;

                double directionRad = Math.atan2(yP - y, xP - x);
                baseRobot.driveModule.setExtrinsicTargetPower(powerP, new Angle(directionRad, Angle.Unit.RADIANS));
                baseRobot.headingLockModule.setTargetHeading(new Angle(headingP, Angle.Unit.RADIANS));

                curActionSet.runUpTo(closestT, baseRobot);
            }
        }
    }

    public synchronized void follow(PurePursuitPath path, ActionSet actionSet) {
        if (isBusy) {
            reset();
        }
        curPath = path;
        curActionSet = actionSet;
        isBusy = true;
    }

    public synchronized void reset() {
        isBusy = false;
        curPath = null;
        curLookaheadWindowStartInches = 0;
        lastPower = 1;
        lastClosestT = 0;
        lastLookaheadT = 0;

        curActionSet = null;
    }

    public synchronized boolean isBusy() {
        return isBusy;
    }

    public synchronized double getLastClosestT() {
        return lastClosestT;
    }

    public synchronized double getLastLookaheadT() {
        return lastLookaheadT;
    }
}
