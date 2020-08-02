package org.quixilver8404.feedforward;

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
}
