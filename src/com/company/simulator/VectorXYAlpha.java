package com.company.simulator;

import org.apache.commons.math3.linear.ArrayRealVector;

public class VectorXYAlpha {

    public final double x;
    public final double y;
    public final double phi;

    public final double sin;
    public final double cos;
    public final double D;

    public VectorXYAlpha(final double x, final double y, final double phi) {
        this.x = x;
        this.y = y;
        this.phi = phi;
        sin = Math.sin(phi);
        cos = Math.cos(phi);
        D = Math.hypot(x, y);
    }

    public static VectorXYAlpha AddVector(final VectorXYAlpha v1, final VectorXYAlpha v2) {
        return new VectorXYAlpha(v1.x+v2.x, v1.y+v2.y, v1.phi +v2.phi);
    }

    public static VectorXYAlpha SubtractVector(final VectorXYAlpha v1, final VectorXYAlpha v2) {
        return new VectorXYAlpha(v1.x+v2.x, v1.y+v2.y, v1.phi +v2.phi);
    }

    public String toString() {return "[" + x + "," + y + "," + phi + "]";}

    public double[] toArray() {return new double[] {x,y, phi}; }

    public ArrayRealVector toRealVector() {return new ArrayRealVector(toArray()); }

    public VectorXYAlpha scalarMultiply(final double s) { return new VectorXYAlpha(x*s, y*s, phi *s); }

    public static VectorXYAlpha fromArray(final double[] a) {
        if (a.length != 3) {
            throw new IndexOutOfBoundsException("There must be three parameters in a VectorXYAlpha");
        }
        return new VectorXYAlpha(a[0], a[1], a[2]);
    }
}
