package org.quixilver8404.feedforward;

public class ConnectionPoint extends Point2D {
    public Segment prevSegment = null;
    public Segment nextSegment = null;
    public boolean complete;
    protected double minVelocity;
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
    }

    public double getMinVelocity() {
        return minVelocity;
    }
}
