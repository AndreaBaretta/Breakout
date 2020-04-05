package com.company.simulator;

import org.apache.commons.math3.linear.ArrayRealVector;

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

    public String toString() {return "[" + x + "," + y + "," + theta + "]";}

    public double[] toArray() {return new double[] {x,y,theta}; }

    public Vector3 scalarMultiply(final double s) { return new Vector3(x*s, y*s, theta*s); }

    public ArrayRealVector toRealVector() {return new ArrayRealVector(toArray()); }
}