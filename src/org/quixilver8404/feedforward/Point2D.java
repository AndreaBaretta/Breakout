package org.quixilver8404.feedforward;

import org.quixilver8404.util.Config;

public class Point2D {
    public final double x;
    public final double y;

    public Point2D(final double x, final double y) {
        this.x = x;
        this.y = y;
    }

    public Point2D copy() {
        return new Point2D(x, y);
    }

    public String toString() {
        return "(" + x/ Config.INCHES_TO_METERS + ", " + y/Config.INCHES_TO_METERS + ")";
    }
}
