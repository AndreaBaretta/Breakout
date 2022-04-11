package org.quixilver8404.breakout.util;

import org.apache.commons.math3.linear.ArrayRealVector;

public class Vector3 {

    public double x;
    public double y;
    public double theta;

    public Vector3() {
        x = y = theta = 0;
    }

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
        final double theta = angleFromSinCos(sin, cos);
        if (Math.abs(theta) > Math.PI) {
            throw new Error("Weee woo: theta="+theta);
        }
        return new Vector3(v1.x - v2.x, v1.y - v2.y, theta);
    }

    public static double normalizeAlpha(final double alpha) {
        if (0 <= alpha && alpha < 2*Math.PI) return alpha;
        final double sin = Math.sin(alpha);
        final double cos = Math.cos(alpha);
        final double angle_from_minus_pi_to_pi = angleFromSinCos(sin, cos);
        if (0 <= angle_from_minus_pi_to_pi) {
            return angle_from_minus_pi_to_pi;
        } else {
            return angle_from_minus_pi_to_pi + 2*Math.PI;
        }
    }

    public static double angleFromSinCos(final double sin, final double cos) {
        return Math.atan2(sin, cos);
    }

    public String toString() {return "[" + x + "," + y + "," + theta + "]";}

    public double[] toArray() {return new double[] {x,y,theta}; }

    public Vector3 scalarMultiply(final double s) { return new Vector3(x*s, y*s, theta*s); }

    public ArrayRealVector toRealVector() { return new ArrayRealVector(toArray()); }

    public void setX(final double x) {
        this.x = x;
    }

    public void setY(final double y) {
        this.y = y;
    }

    public void setTheta(final double theta) {
        this.theta = theta;
    }

    public static Vector3 AddVector(final Vector3 v1, final Vector3 v2) {
        return new Vector3(v1.x+v2.x, v1.y+v2.y, v1.theta+v2.theta);
    }

    public static Vector3 ScalarMultiply(final Vector3 v1, final double a) {
        return new Vector3(v1.x * a, v1.y*a, v1.theta*a);
    }
}