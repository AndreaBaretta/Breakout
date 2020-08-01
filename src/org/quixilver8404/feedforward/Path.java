package org.quixilver8404.feedforward;

import org.quixilver8404.simulator.PowerProfile;
import org.quixilver8404.simulator.Vector3;

import java.util.ArrayList;
import java.util.List;

public class Path {
    public final List<AnchorPoint> anchorPoints;
    public final List<MainSegment> mainSegments;

    protected MainSegment currentSegment;
    protected boolean finished;

    public Path(final List<AnchorPoint> anchorPoints) {
        this.anchorPoints = anchorPoints;
        this.mainSegments = new ArrayList<MainSegment>();

        AnchorPoint curPoint = null;
        AnchorPoint prevPoint = curPoint;

        double s0 = 0;
        double connectionPointCounter = 0;
        for (int i = 0; i < anchorPoints.size(); i++) {
            System.out.println(i);
            curPoint = anchorPoints.get(i);
            if (curPoint.first) {
                prevPoint = curPoint;
            } else {
                final ConnectionPoint connection0 = prevPoint.middlePoint;
                final ConnectionPoint connection1 = prevPoint.nextPoint;
                final ConnectionPoint connection2 = curPoint.prevPoint;
                final ConnectionPoint connection3 = curPoint.middlePoint;

                connection0.index = connectionPointCounter;
                connectionPointCounter++;
                connection1.index = connectionPointCounter;
                connectionPointCounter++;
                connection2.index = connectionPointCounter;
                connectionPointCounter++;
                connection3.index = connectionPointCounter;
                connectionPointCounter++;

                System.out.println("toString: " + prevPoint.center1.toString());
                final double prevAnchorTheta = Math.atan2(prevPoint.middlePoint.y-prevPoint.center1.y, prevPoint.middlePoint.x-prevPoint.center1.x);
                final double curAnchorTheta = Math.atan2(curPoint.middlePoint.y-curPoint.center0.y, curPoint.middlePoint.x-curPoint.center0.x);

                final CircleSegment segment0 = new CircleSegment(connection0, connection1, s0, prevPoint.configVelocity, prevPoint.center1, prevPoint.r1, prevAnchorTheta, prevPoint.theta1, prevPoint.counterClockwise1);
                s0 = segment0.getEndS();
                final LinearSegment segment1 = new LinearSegment(connection1, connection2, s0, prevPoint.configVelocity);
                s0 = segment1.getEndS();
                final CircleSegment segment2 = new CircleSegment(connection2, connection3, s0, prevPoint.configVelocity, curPoint.center0, curPoint.r0, curPoint.theta0, curAnchorTheta, curPoint.counterClockwise0);
                s0 = segment2.getEndS();

                final MainSegment mainSegment = new MainSegment(segment0, segment1, segment2, i - 1, prevPoint, curPoint);

                mainSegments.add(mainSegment);

                prevPoint = curPoint;
//                break;
            }
        }

        currentSegment = mainSegments.get(0);
        finished = false;
    }

    public RobotState evaluate(final double s, final double s_dot, final  double s_dot_dot) {
        final double endS = currentSegment.getEndS();
        if (finished) { //If at end, stay there
            return new RobotState(
                    currentSegment.getPosition(endS),
                    currentSegment.getVelocity(endS, 0),
                    currentSegment.getAcceleration(endS, 0, 0)
            );
        }
        if (currentSegment.index == mainSegments.size() - 1 && s >= endS - 0.01) { //Condition to finish
            finish();
            return new RobotState(
                    currentSegment.getPosition(endS),
                    currentSegment.getVelocity(endS, 0),
                    currentSegment.getAcceleration(endS, 0, 0)
            );
        } else if (s < 0) {
            return new RobotState(
                    currentSegment.getPosition(currentSegment.s0),
                    currentSegment.getVelocity(currentSegment.s0, s_dot),
                    currentSegment.getAcceleration(currentSegment.s0, s_dot, s_dot_dot)
            );
        } else { //Else, do the normal thing
            return new RobotState(
                    currentSegment.getPosition(s),
                    currentSegment.getVelocity(s, s_dot),
                    currentSegment.getAcceleration(s, s_dot, s_dot_dot)
                );
        }
    }

    public double calcS(final double x, final double y) {
        final double curSegmentS = currentSegment.calcS(x, y);
        if (curSegmentS >= currentSegment.getEndS()) {
            next();
            return currentSegment.calcS(x, y);
        } else if (curSegmentS < currentSegment.s0) {
            back();
            return currentSegment.calcS(x, y);
        } else {
            return curSegmentS;
        }
    }

    protected void next() {
        if (currentSegment.index == mainSegments.size() - 1) {
            ;
        } else {
            currentSegment = mainSegments.get(currentSegment.index + 1);
        }
    }

    protected void back() {
        if (currentSegment.index == 0) {
            ;
        } else {
            currentSegment = mainSegments.get(currentSegment.index - 1);
        }
    }

    protected void finish() {
        finished = true;
    }

    public double calcAccelerationCorrection(final double s, final double s_dot) {
        double s_dot_dot = Config.MAX_ACCELERATION;
        while (true) {
            final RobotState state = evaluate(s, s_dot, s_dot_dot);
            final Vector3 vel = state.vel;
            final Vector3 acc = state.acc;
            final double[] powerSettings = PowerProfile.toRawPowerSettings(acc, vel, state.pos.theta);
//            final double[] powerSettings = PowerProfile.toRawPowerSettings(new Vector3(0,0,0), vel, state.pos.theta);
            boolean optimized = true;
//            System.out.println("s_dot_dot: " + s_dot_dot + "  Power settings: " + Arrays.toString(powerSettings) + "  s: " + s + "  s_dot: " + s_dot + "  alpha: " + state.pos.theta + "  acceleration: " + state.acc.toString());
            for (double p : powerSettings) {
                if (Math.abs(p) >= 1) {
                    optimized = false;
                }
            }
            if (optimized) {
                return s_dot_dot;
            }
            if (s_dot_dot < Config.MAX_DECELERATION) {
                return 0;
            }
            s_dot_dot -= Config.ACCELERATION_CORRECTION_STEP;
        }
    }
}
