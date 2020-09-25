package org.quixilver8404.breakout.feedforward;

import org.quixilver8404.breakout.util.Config;

public abstract class Point {
    final double x;
    final double y;
    final double tan;

    Point(final double x, final double y, final double tan) {
        this.x = x;
        this.y = y;
        this.tan = tan;
    }

    Point(final Point2D point, final double tan) {
        this.x = point.x;
        this.y = point.y;
        this.tan = tan;
    }

    public String toString() {
        return "(" + x/ Config.INCHES_TO_METERS + ", " + y/Config.INCHES_TO_METERS + ")";
    }
}
