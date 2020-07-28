package com.company.feedforward;

import com.company.simulator.Vector3;

public class MainSegment extends Segment {
    public final CircleSegment circleSegment0;
    public final LinearSegment linearSegment;
    public final CircleSegment circleSegment1;

    public final int index;

    protected Segment currentSegment;

    MainSegment(final CircleSegment circleSegment0, final LinearSegment linearSegment, final CircleSegment circleSegment1, final int index) {
        super(circleSegment0.firstPoint, circleSegment1.lastPoint, circleSegment0.s0, Config.MAX_VELOCITY);
        this.circleSegment0 = circleSegment0;
        this.linearSegment = linearSegment;
        this.circleSegment1 = circleSegment1;
        this.index = index;
        currentSegment = circleSegment0;
    }

    public Vector3 getPosition(final double s) {
        if (circleSegment0.inRange(s)) {
            return circleSegment0.getPosition(s);
        } else if (linearSegment.inRange(s)) {
            return linearSegment.getPosition(s);
        } else {
            return circleSegment1.getPosition(s);
        }
    }

    public Vector3 getVelocity(final double s, final double s_dot) {
        if (circleSegment0.inRange(s)) {
            return circleSegment0.getVelocity(s, s_dot);
        } else if (linearSegment.inRange(s)) {
            return linearSegment.getVelocity(s, s_dot);
        } else {
            return circleSegment1.getVelocity(s, s_dot);
        }
    }

    public Vector3 getAcceleration(final double s, final double s_dot, final double s_dot_dot) {
        if (circleSegment0.inRange(s)) {
            return circleSegment0.getAcceleration(s, s_dot, s_dot_dot);
        } else if (linearSegment.inRange(s)) {
            return linearSegment.getAcceleration(s, s_dot, s_dot_dot);
        } else {
            return circleSegment1.getAcceleration(s, s_dot, s_dot_dot);
        }
    }

    public double getTotalS() {
        return circleSegment0.getTotalS() + linearSegment.getTotalS() + circleSegment1.getTotalS();
    }

    public double calcMinVelocity() {
        return Config.MAX_VELOCITY;
    }

    public double calcS(final double x, final double y) {
        if (currentSegment == circleSegment0) {
            if (circleSegment0.calcS(x, y) >= circleSegment0.getEndS()) {
                currentSegment = linearSegment;
            }
            return currentSegment.calcS(x, y);
        } else if (currentSegment == linearSegment) {
            if (linearSegment.calcS(x, y) >= linearSegment.getEndS()) {
                currentSegment = circleSegment1;
            }
            return currentSegment.calcS(x, y);
        } else {
            return currentSegment.calcS(x, y);
        }
    }
}
