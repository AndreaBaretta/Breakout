package com.company.feedforward;

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
}
