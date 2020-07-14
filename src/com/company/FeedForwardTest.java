package com.company;

import com.company.simulator.Vector3;

public class FeedForwardTest {

    public final double a = -7.6;
    public final double b = 3.95;

    public Vector3 getPosition(final double t) {
//        return new Vector3(0, 0, 0);
//        return new Vector3(Math.cos(t), 0, 0);
//        return new Vector3(Math.pow(Math.cos(t),3), Math.pow(Math.sin(t),3),t);
        return new Vector3(Math.cos(0.5*t), Math.sin(0.5*t), t);
//        return new Vector3(0, t, 0);
//        return new Vector3(t, 0, 0);
//        return new Vector3(
//                (a+b)*Math.cos(t)-b*Math.cos((1+a/b)*t),
//                (a+b)*Math.sin(t)-b*Math.sin((1+a/b)*t),
//                t
//        );
    }

    public Vector3 getVelocity(final double t) {
//        return new Vector3(0, 0,0);
//        return new Vector3(-Math.sin(t), 0,0);
//        return new Vector3(-3*Math.pow(Math.cos(t),2)*Math.sin(t), 3*Math.pow(Math.sin(t),2)*Math.cos(t),1);
        return new Vector3(-0.5*Math.sin(0.5*t), 0.5*Math.cos(0.5*t), 1);
//        return new Vector3(0, 1, 0);
//        return new Vector3(1, 0, 0);
//        return new Vector3(
//                -(a+b)*Math.sin(t)+b*(1+a/b)*Math.sin((1+a/b)*t),
//                (a+b)*Math.cos(t)-b*(1+a/b)*Math.cos((1+a/b)*t),
//                1
//        );
    }

    public Vector3 getAcceleration(final double t) {
//        return new Vector3(0, 0,0);
//        return new Vector3(-Math.cos(t), 0,0);
//        return new Vector3(-3*Math.pow(Math.cos(t),3) + 6*Math.cos(t)*Math.pow(Math.sin(t),2), -3*Math.pow(Math.sin(t),3) + 6*Math.sin(t)*Math.pow(Math.cos(t),2),0);
        return new Vector3(-0.25*Math.cos(0.5*t), -0.25*Math.sin(0.5*t), 0);
//        return new Vector3(0,0,0);
//        return new Vector3(
//                -(a+b)*Math.cos(t)+b*Math.pow((1+a/b),2)*Math.cos((1+a/b)*t),
//                -(a+b)*Math.sin(t)+b*Math.pow((1+a/b),2)*Math.sin((1+a/b)*t),
//                1
//        );
    }
}
