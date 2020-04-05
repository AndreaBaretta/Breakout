package com.company.simulator;

import org.apache.commons.math3.linear.ArrayRealVector;

public class VectorXYAlpha {

    public final double x;
    public final double y;
    public final double alpha;

    public VectorXYAlpha(final double x, final double y, final double alpha) {
        this.x = x;
        this.y = y;
        this.alpha = alpha;
    }

    public static VectorXYAlpha AddVector(final VectorXYAlpha v1, final VectorXYAlpha v2) {
        return new VectorXYAlpha(v1.x+v2.x, v1.y+v2.y, v1.alpha +v2.alpha);
    }

    public static VectorXYAlpha SubtractVector(final VectorXYAlpha v1, final VectorXYAlpha v2) {
        return new VectorXYAlpha(v1.x+v2.x, v1.y+v2.y, v1.alpha +v2.alpha);
    }

    public String toString() {return "[" + x + "," + y + "," + alpha + "]";}

    public double[] toArray() {return new double[] {x,y, alpha}; }

    public ArrayRealVector toRealVector() {return new ArrayRealVector(toArray()); }

    public VectorXYAlpha scalarMultiply(final double s) { return new VectorXYAlpha(x*s, y*s, alpha *s); }

    public static VectorXYAlpha fromArray(final double[] a) {
        if (a.length != 3) {
            throw new IndexOutOfBoundsException("There must be three parameters in a VectorXYAlpha");
        }
        return new VectorXYAlpha(a[0], a[1], a[2]);
    }
}
