package org.quixilver8404.breakout.feedforward;

import org.json.simple.JSONObject;
import org.quixilver8404.breakout.util.Config;
import org.quixilver8404.breakout.util.Vector3;

public class CurveParameters {
    protected final double circle1Radius;
    protected final double circle2Radius;
    protected final Point2D circle1Center;
    protected final Point2D circle2Center;
    protected final double endTheta1;
    protected final double endTheta2;
    protected final Point2D p1;
    protected final Point2D p2;

    CurveParameters(final JSONObject curve) {
        circle1Radius = (double)curve.get("circle1Radius")* Config.INCHES_TO_METERS;
        circle2Radius = (double)curve.get("circle2Radius")*Config.INCHES_TO_METERS;
        circle1Center = new Point2D((double)curve.get("circle1X")*Config.INCHES_TO_METERS, (double)curve.get("circle1Y")*Config.INCHES_TO_METERS);
        circle2Center = new Point2D((double)curve.get("circle2X")*Config.INCHES_TO_METERS, (double)curve.get("circle2Y")*Config.INCHES_TO_METERS);
        endTheta1 = Vector3.normalizeAlpha((double)curve.get("endTheta1"));
        endTheta2 = Vector3.normalizeAlpha((double)curve.get("endTheta2"));
        p1 = new Point2D((double)curve.get("p1X")*Config.INCHES_TO_METERS, (double)curve.get("p1Y")*Config.INCHES_TO_METERS);
        p2 = new Point2D((double)curve.get("p2X")*Config.INCHES_TO_METERS, (double)curve.get("p2Y")*Config.INCHES_TO_METERS);
    }

    CurveParameters(final double circle1Radius, final double circle2Radius, final Point2D circle1Center, final Point2D circle2Center,
                    final double endTheta1, final double endTheta2, final Point2D p1, final Point2D p2) {
        this.circle1Radius = circle1Radius;
        this.circle2Radius = circle2Radius;
        this.circle1Center = circle1Center;
        this.circle2Center = circle2Center;
        this.endTheta1 = endTheta1;
        this.endTheta2 = endTheta2;
        this.p1 = p1;
        this.p2 = p2;
    }

    protected CurveParameters copy() {
        return new CurveParameters(circle1Radius, circle2Radius, circle1Center.copy(), circle2Center.copy(),
                endTheta1, endTheta2, p1.copy(), p2.copy());
    }
}
