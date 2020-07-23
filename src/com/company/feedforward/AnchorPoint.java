package com.company.feedforward;

public class AnchorPoint extends Point {
    public enum Heading {
        FRONT, BACK, CUSTOM
    }

    final Heading heading;
    final double customHeading;
    final double r0;
    final Point2D center0;
    final double theta0;
    final double r1;
    final Point2D center1;
    final double theta1;

    final ConnectionPoint leftPoint;
    final ConnectionPoint centerPoint;
    final ConnectionPoint rightPoint;

    final boolean counterClockwise0;
    final boolean counterClockwise1;

    public boolean CheckDirection(final Point2D center) {
        if (x > center.x && y >= center.y) { //Q1
            if (Math.PI/2 <= tan && tan <= Math.PI) {
                return true;
            } else {
                return false;
            }
        } else if (x <= center.x && y > center.y) { //Q2
            if (Math.PI <= tan && tan <= 3*Math.PI/2) {
                return true;
            } else {
                return false;
            }
        } else if (x < center.x && y <= center.y) { //Q3
            if (3*Math.PI/2 <= tan && tan <= 2*Math.PI) {
                return true;
            } else {
                return false;
            }
        } else {
            if (0 <= tan && tan <= Math.PI/2) { //Q4
                return true;
            } else {
                return false;
            }
        }
    }

    AnchorPoint(final double x, final double y, final double tan, final Heading heading, final double customHeading,
                final double r0, final Point2D center0, final double theta0, final double r1, final Point2D center1, final double theta1,
                final Point2D tanPoint0, final Point2D tanPoint1) {
        super(x, y, tan);
        this.heading = heading;
        if (heading == Heading.CUSTOM) {
            this.customHeading = customHeading;
        } else if (heading == Heading.FRONT) {
            this.customHeading = tan;
        } else {
            this.customHeading = -tan;
        }
        this.r0 = r0;
        this.center0 = center0;
        this.theta0 = theta0;
        this.r1 = r1;
        this.center1 = center1;
        this.theta1 = theta1;

        leftPoint = new ConnectionPoint(tanPoint0);
        centerPoint = new ConnectionPoint(x, y);
        rightPoint = new ConnectionPoint(tanPoint1);

        counterClockwise0 = CheckDirection(center0);
        counterClockwise1 = CheckDirection(center1);
    }
}
