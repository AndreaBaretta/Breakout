package org.quixilver8404.feedforward;

import org.quixilver8404.util.Config;

public class ConnectionPoint extends Point2D implements VelocityPoint, HeadingPoint {
    public MinorSegment prevSegment = null;
    public MinorSegment nextSegment = null;
    public boolean complete;
    protected double minVelocity;
    public double configVelocity;
    protected double s;
    public double index;
    public final AnchorPoint.Heading headingState;
    public final double heading;

    ConnectionPoint(final double x, final double y, final AnchorPoint.Heading headingState, final double heading, final double configVelocity) {
        super(x, y);
        this.heading = heading;
        this.headingState = headingState;
        complete = false;
        minVelocity = configVelocity;
        this.configVelocity = configVelocity;
    }

    ConnectionPoint(final Point2D point, final AnchorPoint.Heading headingState, final double heading, final double configVelocity) {
        super(point.x, point.y);
        this.heading = heading;
        this.headingState = headingState;
        complete = false;
        minVelocity = configVelocity;
        this.configVelocity = configVelocity;
    }

    public void setPrevSegment(final MinorSegment segment) {
        if (complete) {
            throw new Error("Error parsing: Connector point already complete when setting prevSegment at index: " + index);
        }
        prevSegment = segment;
        if (prevSegment != null && nextSegment != null) {
            complete = true;
        }
//        System.out.println("Set prevSegment at index: " + index + "  s = " + segment.getEndS());
        s = segment.getEndS();
    }

    public void setNextSegment(final MinorSegment segment) {
        if (complete) {
            throw new Error("Error parsing: Connector point already complete when setting nextSegment at index: " + index);
        }
        nextSegment = segment;
        if (prevSegment != null && nextSegment != null) {
            complete = true;
        }
        minVelocity = Math.min(minVelocity, nextSegment.getMinVelocity());
//        System.out.println("Set nextSegment at index: " + index + "  s = " + segment.s0);
        s = segment.s0;
    }

    public double getMinVelocity() {
        return minVelocity;
    }

    public double getConfigVelocity() {
        return configVelocity;
    }

    public void setConfigVelocity(final double newConfigVelocity) {
        configVelocity = newConfigVelocity;
    }

    public double getS() {
        return s;
    }

    public double getHeading() {
        return heading;
    }

    public AnchorPoint.Heading getHeadingState() {
        return headingState;
    }

    public String toString() {
        return "(" + x/Config.INCHES_TO_METERS + ", " + y/Config.INCHES_TO_METERS + ", s=" + s + ", configVelocity: " + configVelocity + ", minVelocity: " + minVelocity + ")";
    }

}
