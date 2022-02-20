package org.quixilver8404.breakout.feedforward;

import org.quixilver8404.breakout.util.Config;
import org.quixilver8404.breakout.util.Vector3;

public class LinearSegment extends Segment {

    public final double theta;
    public final double x0;
    public final double y0;
    public final double x1;
    public final double y1;

    public LinearSegment(final ConnectionPoint firstPoint, final ConnectionPoint lastPoint, final double s0) {
        super(firstPoint, lastPoint, s0);
        theta = Math.atan2(lastPoint.y-firstPoint.y, lastPoint.x-firstPoint.x);
        x0 = firstPoint.x;
        y0 = firstPoint.y;
        x1 = lastPoint.x;
        y1 = lastPoint.y;
    }

    public Vector3 getPosition(final double s) {
        return new Vector3(
                x0 + (s - s0)*Math.cos(theta),
                y0 + (s - s0)*Math.sin(theta),
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
        return Math.hypot(x1-x0, y1-y0);
    }

    public double getMaxVelocity() {
        return Config.MAX_VELOCITY*Config.MAX_SAFE_VELOCITY;
    }

    public double calcS(final double x, final double y) {
        final double[] pos = new double[]{x - x0, y - y0};
        final double[] u_s = new double[]{(x1 - x0)/getTotalS(), (y1 - y0)/getTotalS()};
        return pos[0]*u_s[0] + pos[1]*u_s[1] + s0;
    }

    public boolean isPointSegment() {
        return false;
    }

    public String toString() {
        return "(type=line, s0=" + s0 + ", s1=" + getEndS() + ", i=" + index + ", isZero=" + zeroSegment + ")";
    }
}
