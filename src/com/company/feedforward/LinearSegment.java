package com.company.feedforward;

import com.company.simulator.Vector3;

public class LinearSegment extends Segment {

    public final double theta;

    LinearSegment(final ConnectionPoint firstPoint, final ConnectionPoint lastPoint, final double s0, final double configVelocity) {
        super(firstPoint, lastPoint, s0, configVelocity);
        theta = Math.atan2(lastPoint.y-firstPoint.y, lastPoint.x-firstPoint.x);
    }

    public Vector3 getPosition(final double s) {
        return new Vector3(
                firstPoint.x + (s - s0)*Math.cos(theta),
                firstPoint.y + (s - s0)*Math.sin(theta),
                theta
        );
    }

    public Vector3 getVelocity(final double s, final double s_dot) {
        return new Vector3(
                s_dot*Math.cos(theta),
                s_dot*Math.sin(theta),
                0
        );
    }

    public Vector3 getAcceleration(final double s, final double s_dot, final double s_dot_dot) {
        return new Vector3(
                s_dot_dot*Math.cos(theta),
                s_dot_dot*Math.sin(theta),
                0
        );
    }

    public double getTotalS() {
        return Math.hypot(lastPoint.x-firstPoint.x, lastPoint.y-firstPoint.y);
    }

    public double calcMinVelocity() {
        return minVelocity;
    }

    public double calcS(final double x, final double y) {
        final double[] pos = new double[]{x - firstPoint.x, y - firstPoint.y};
        final double[] u_s = new double[]{(lastPoint.x - firstPoint.x)/getTotalS(), (lastPoint.y - firstPoint.y)/getTotalS()};
        return pos[0]*u_s[0] + pos[1]*u_s[1] + s0;
    }
}
