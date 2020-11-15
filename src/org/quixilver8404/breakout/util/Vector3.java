package org.quixilver8404.breakout.util;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.quixilver8404.breakout.feedforward.MainSegment;

public class Vector3 {

    public final double x;
    public final double y;
    public final double theta;

    public Vector3(final double x, final double y, final double theta) {
        this.x = x;
        this.y = y;
        this.theta = theta;
    }

    public static Vector3 addVector(final Vector3 v1, final Vector3 v2) {
        return new Vector3(v1.x+v2.x, v1.y+v2.y, v1.theta+v2.theta);
    }

    public static Vector3 subtractVector(final Vector3 v1, final Vector3 v2) {
        return new Vector3(v1.x-v2.x, v1.y-v2.y, v1.theta-v2.theta);
    }

    public static Vector3 subtractVector2(final Vector3 v1, final Vector3 v2) {
        final double sin = Math.sin(v1.theta)*Math.cos(v2.theta) - Math.cos(v1.theta)*Math.sin(v2.theta);
        final double cos = Math.cos(v1.theta)*Math.cos(v2.theta) + Math.sin(v1.theta)*Math.sin(v2.theta);
        final double theta = MainSegment.angleFromSinCos(sin, cos);
        if (Math.abs(theta) > Math.PI) {
            throw new Error("Weee woo: theta="+theta);
        }
        return new Vector3(v1.x - v2.x, v1.y - v2.y, theta);
    }

    public String toString() {return "[" + x + "," + y + "," + theta + "]";}

    public double[] toArray() {return new double[] {x,y,theta}; }

    public Vector3 scalarMultiply(final double s) { return new Vector3(x*s, y*s, theta*s); }

    public ArrayRealVector toRealVector() { return new ArrayRealVector(toArray()); }
}