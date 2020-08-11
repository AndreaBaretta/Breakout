package org.quixilver8404.util;

import org.quixilver8404.feedforward.ActionEventListener;
import org.quixilver8404.feedforward.ActionFunction;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class Config {
    public final static double MAX_ACCELERATION = 11.2;
    public final static double MAX_DECELERATION = -11.2;
    public final static double MAX_VELOCITY = 1.17749;
    public final static double ACCELERATION_CORRECTION_STEP = 0.01;
    public final static double T_MAX = 2.1;
    public final static double OMEGA_MAX = 31.4;
    public final static double MASS = 20;
    public final static double WHEEL_RADIUS = 37.5/1000;
    public final static double J = 0.8333333333333333;
    public final static double r_X = 0.4572/2;
    public final static double r_Y = 0.4572/2;
    public final static double INCHES_TO_METERS = 0.0254;
    public final static double MAX_SAFE_VELOCITY = 0.95;
    public final static double MAX_SAFE_ACCELERATION = 0.95;
    public final static double ACCELERATION_CORRECTION = Math.PI/2.5;

    public final static List<ActionEventListener> actionEventListeners = Arrays.asList(new ActionEventListener[]{
            new ActionEventListener(1, new ActionFunction() {
                @Override
                public void run() {
                    System.out.println("1 executed");
                }
            }),
            new ActionEventListener(2, new ActionFunction() {
                @Override
                public void run() {
                    System.out.println("2 executed");
                }
            }),
            new ActionEventListener(3, new ActionFunction() {
                @Override
                public void run() {
                    System.out.println("3 executed");
                }
            })
    });
}