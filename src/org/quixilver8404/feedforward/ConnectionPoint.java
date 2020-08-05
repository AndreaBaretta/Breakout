package org.quixilver8404.feedforward;

import org.quixilver8404.util.Config;

public class ConnectionPoint extends Point2D implements VelocityPoint {
    public Segment prevSegment = null;
    public Segment nextSegment = null;
    public boolean complete;
    protected double minVelocity;
    protected double s;
    public double index;

    ConnectionPoint(final double x, final double y, final double configVelocity) {
        super(x, y);
        complete = false;
        minVelocity = configVelocity;
    }

    ConnectionPoint(final Point2D point, final double configVelocity) {
        super(point.x, point.y);
        complete = false;
        minVelocity = configVelocity;
    }

    public void setPrevSegment(final Segment segment) {
        if (complete) {
            throw new Error("Error parsing: Connector point already complete when setting prevSegment at index: " + index);
        }
        prevSegment = segment;
        if (prevSegment != null && nextSegment != null) {
            complete = true;
        }
        minVelocity = Math.min(minVelocity, prevSegment.minVelocity);
        System.out.println("Set prevSegment at index: " + index + "  s = " + segment.getEndS());
        s = segment.getEndS();
    }

    public void setNextSegment(final Segment segment) {
        if (complete) {
            throw new Error("Error parsing: Connector point already complete when setting nextSegment at index: " + index);
        }
        nextSegment = segment;
        if (prevSegment != null && nextSegment != null) {
            complete = true;
        }
        minVelocity = Math.min(minVelocity, nextSegment.minVelocity);
        System.out.println("Set nextSegment at index: " + index + "  s = " + segment.s0);
        s = segment.s0;
    }

    public double getMinVelocity() {
        return minVelocity;
    }

    public double getS() {
        return s;
    }

    public String toString() {
        return "(" + x/Config.INCHES_TO_METERS + ", " + y/Config.INCHES_TO_METERS + ", s=" + s + ")";
    }
}
