package com.company.feedforward;

import com.company.simulator.Vector3;

public class MainSegment extends Segment {
    final CircleSegment circleSegment0;
    final LinearSegment linearSegment;
    final CircleSegment circleSegment1;

    MainSegment(final CircleSegment circleSegment0, final LinearSegment linearSegment, final CircleSegment circleSegment1) {
        super(circleSegment0.firstPoint, circleSegment1.lastPoint, circleSegment0.s0);
        this.circleSegment0 = circleSegment0;
        this.linearSegment = linearSegment;
        this.circleSegment1 = circleSegment1;
    }

    public Vector3 getPosition(final double s) {
        return new Vector3(0,0);
    }

    public Vector3 getVelocity(final double s, final double s_dot) {
        return new Vector3(0,0);
    }

    public Vector3 getAcceleration(final double s, final double s_dot, final double s_dot_dot) {
        return new Vector3(0,0);
    }
}
