package com.company.quixilver8404.skystone.control.math;

import com.company.quixilver8404.skystone.util.measurement.Angle;
import com.company.quixilver8404.skystone.util.measurement.Distance;
import com.company.quixilver8404.skystone.util.measurement.Pose2D;

public class Vector3 {

    public final double x;
    public final double y;
    public final double theta;

    public Vector3(final double x, final double y, final double theta) {
        this.x = x;
        this.y = y;
        this.theta = theta;
    }

    public static Vector3 AddVector(final Vector3 v1, final Vector3 v2) {
        return new Vector3(v1.x+v2.x, v1.y+v2.y, v1.theta+v2.theta);
    }

    public static Vector3 SubtractVector(final Vector3 v1, final Vector3 v2) {
        return new Vector3(v1.x+v2.x, v1.y+v2.y, v1.theta+v2.theta);
    }

    public String toString() {return "[" + x + "," + y + "," + theta + "]";}

    public double[] toArray() {return new double[] {x,y,theta}; }

//    public ArrayRealVector toRealVector() {return new ArrayRealVector(toArray()); }

    public static Vector3 fromPose2D(final Pose2D pose2D) {
        final Distance.Unit millimeters = Distance.Unit.MILLIMETERS;
        return new Vector3(pose2D.x.getValue(millimeters), pose2D.y.getValue(millimeters), pose2D.heading.getStandard(Angle.Unit.RADIANS));
    }

    public Pose2D toPose2D() {
        return new Pose2D(new Distance(x, Distance.Unit.MILLIMETERS), new Distance(y, Distance.Unit.MILLIMETERS), new Angle(theta, Angle.Unit.RADIANS));
    }
}
