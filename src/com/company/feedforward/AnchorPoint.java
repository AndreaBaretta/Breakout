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

    final ConnectionPoint prevPoint;
    final ConnectionPoint middlePoint;
    final ConnectionPoint nextPoint;

    final boolean counterClockwise0;
    final boolean counterClockwise1;

    public final double configVelocity;

    final boolean first;
    final boolean last;

    public boolean CheckDirection(final Point2D center) {
        if (center == null) {
            return false;
        }
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

    public AnchorPoint(final double x, final double y, final double tan, final Heading heading, final double customHeading,
                final double r0, final Point2D center0, final double theta0, final double r1, final Point2D center1, final double theta1,
                final Point2D tanPoint0, final Point2D tanPoint1, final double configVelocity, final boolean first, final boolean last) {
        super(x, y, tan);
        this.heading = heading;
        if (heading == Heading.CUSTOM) {
            this.customHeading = customHeading;
        } else if (heading == Heading.FRONT) {
            this.customHeading = tan;
        } else {
            this.customHeading = -tan;
        }

        final Point2D tanPoint0_;
        final Point2D tanPoint1_;

        if (first) {
            this.r0 = Double.NaN;
            this.center0 = null;
            this.theta0 = Double.NaN;
            tanPoint0_ = new Point2D(Double.NaN, Double.NaN);
            this.r1 = r1;
            this.center1 = center1;
            this.theta1 = theta1;
            tanPoint1_ = tanPoint1;
        } else if (last) {
            this.r0 = r0;
            this.center0 = center0;
            this.theta0 = theta0;
            tanPoint0_ = tanPoint0;
            this.r1 = Double.NaN;
            this.center1 = null;
            this.theta1 = Double.NaN;
            tanPoint1_ = new Point2D(Double.NaN, Double.NaN);
        } else {
            this.r0 = r0;
            this.center0 = center0;
            this.theta0 = theta0;
            tanPoint0_ = tanPoint0;
            this.r1 = r1;
            this.center1 = center1;
            this.theta1 = theta1;
            tanPoint1_ = tanPoint1;
        }

        prevPoint = new ConnectionPoint(tanPoint0_, configVelocity);
        middlePoint = new ConnectionPoint(x, y, configVelocity);
        nextPoint = new ConnectionPoint(tanPoint1_, configVelocity);

        counterClockwise0 = CheckDirection(center0);
        counterClockwise1 = CheckDirection(center1);

        this.configVelocity = configVelocity;

        this.first = first;
        this.last = last;
    }
}
