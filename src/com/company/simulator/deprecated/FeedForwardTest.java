package com.company.simulator.deprecated;

import com.company.simulator.Vector3;

public class FeedForwardTest {
    public Vector3 getPosition(final double t) {
//        return new Vector3(0, 0, 0);
//        return new Vector3(Math.cos(t), 0, 0);
//        return new Vector3(Math.pow(Math.cos(t),3), Math.pow(Math.sin(t),3),t);
        return new Vector3(Math.cos(t), Math.sin(t), t);
//        return new Vector3(0, t, 0);
//        return new Vector3(t, 0, 0);
    }

    public Vector3 getVelocity(final double t) {
//        return new Vector3(0, 0,0);
//        return new Vector3(-Math.sin(t), 0,0);
//        return new Vector3(-3*Math.pow(Math.cos(t),2)*Math.sin(t), 3*Math.pow(Math.sin(t),2)*Math.cos(t),1);
        return new Vector3(-Math.sin(t), Math.cos(t), 1);
//        return new Vector3(0, 1, 0);
//        return new Vector3(1, 0, 0);
    }

    public Vector3 getAcceleration(final double t) {
//        return new Vector3(0, 0,0);
//        return new Vector3(-Math.cos(t), 0,0);
//        return new Vector3(-3*Math.pow(Math.cos(t),3) + 6*Math.cos(t)*Math.pow(Math.sin(t),2), -3*Math.pow(Math.sin(t),3) + 6*Math.sin(t)*Math.pow(Math.cos(t),2),0);
        return new Vector3(-Math.cos(t), -Math.sin(t), 0);
//        return new Vector3(0,0,0);
    }
}
