package com.company.feedforward;

public class ConnectionPoint extends Point2D {
    protected Segment prevSegment;
    protected Segment nextSegment;
    protected boolean complete;

    ConnectionPoint(final double x, final double y/*, final double tan*/) {
//        super(x, y, tan);
        super(x, y);
        complete = false;
    }

    ConnectionPoint(final Point2D point/*, final double tan*/) {
//        super(point, tan);
        super(point.x, point.y);
        complete = false;
    }

    public void setPrevSegment(final Segment segment) {
        if (complete) {
            throw new Error("Error parsing: Connector point already complete when setting prevSegment");
        }
        prevSegment = segment;
        if (prevSegment != null && nextSegment != null) {
            complete = true;
        }
    }

    public void setNextSegment(final Segment segment) {
        if (complete) {
            throw new Error("Error parsing: Connector point already complete when setting nextSegment");
        }
        nextSegment = segment;
        if (prevSegment != null && nextSegment != null) {
            complete = true;
        }
    }
}
