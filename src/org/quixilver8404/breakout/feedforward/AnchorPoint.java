package org.quixilver8404.breakout.feedforward;

import org.quixilver8404.breakout.util.Vector3;

import java.util.List;
import java.util.Set;

public class AnchorPoint extends Point {

    final HeadingPoint.Heading heading;
    final double customHeading;
    final double r0;
    final Point2D center0;
    final double theta0;
    final double r1;
    final Point2D center1;
    final double theta1;

    final Point2D tanPoint0;
    final Point2D tanPoint1;

    final ConnectionPoint prevPoint;
    final ConnectionPoint middlePoint;
    final ConnectionPoint nextPoint;

    final boolean counterClockwise0;
    final boolean counterClockwise1;

    public final double configVelocity;

    protected boolean first;
    protected boolean last;

    public boolean CheckDirection(final Point2D center) {
        if (center == null) {
            return false;
        }
        final double counterClockTheta = Vector3.normalizeAlpha(Vector3.normalizeAlpha(Math.atan2(y - center.y, x - center.x)) + Math.PI/2);
        final double sin = Math.sin(tan)*Math.cos(counterClockTheta) - Math.cos(tan)*Math.sin(counterClockTheta);
        final double cos = Math.cos(tan)*Math.cos(counterClockTheta) + Math.sin(tan)*Math.sin(counterClockTheta);
        final double theta = Vector3.angleFromSinCos(sin, cos);
        if (Math.abs(theta) <= 1e-12) {
            return true;
        } else {
            return false;
        }
    }

    public AnchorPoint(final double x, final double y, final double tangent, final HeadingPoint.Heading heading, final double customHeading,
                       final double r0, final Point2D center0, final double theta0, final double r1, final Point2D center1, final double theta1,
                       final Point2D tanPoint0, final Point2D tanPoint1, final double configVelocity, final List<ActionEventListener> actionEventListeners,
                       final Set<Integer> actions, final boolean first, final boolean last) {
        super(x, y, Vector3.normalizeAlpha(tangent));
        final double tan = Vector3.normalizeAlpha(tangent);
        this.tanPoint0 = tanPoint0;
        this.tanPoint1 = tanPoint1;
        this.heading = heading;
        if (heading == HeadingPoint.Heading.CUSTOM) {
            this.customHeading = customHeading;
        } else if (heading == HeadingPoint.Heading.FRONT) {
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

        prevPoint = new ConnectionPoint(tanPoint0_, HeadingPoint.Heading.NONE, Double.NaN, Double.NaN, null, null, false);
        final double pointHeading;
        if (heading == HeadingPoint.Heading.CUSTOM) {
            pointHeading = customHeading;
        } else if (heading == HeadingPoint.Heading.FRONT) {
            pointHeading = tan;
        } else {
            pointHeading = Vector3.normalizeAlpha(tan + Math.PI);
        }
        middlePoint = new ConnectionPoint(x, y, heading, pointHeading, configVelocity, actionEventListeners, actions, true);
        nextPoint = new ConnectionPoint(tanPoint1_, HeadingPoint.Heading.NONE, Double.NaN, Double.NaN, null, null, false);

        counterClockwise0 = CheckDirection(center0);
        counterClockwise1 = CheckDirection(center1);

        this.configVelocity = configVelocity;

        this.first = first;
        this.last = last;
    }

    public AnchorPoint copy() {
        return new AnchorPoint(x, y, tan, heading, customHeading, r0, center0, theta0, r1, center1, theta1, tanPoint0, tanPoint1, configVelocity,
                middlePoint.getActionEventListeners(), middlePoint.getActions(), first, last);
    }

    public String toString() {
        return "(" + x + ", " + y + ")";
    }
}
