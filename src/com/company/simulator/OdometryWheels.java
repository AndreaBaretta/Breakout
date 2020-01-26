package com.company.simulator;

public class OdometryWheels {

    public static double[][] makeA(final Vector3[] setup, final double alpha) {
        final double[][] A = new double[][]{
                {Math.cos(alpha + setup[0].theta), Math.sin(alpha + setup[0].theta), Math.hypot(setup[0].x, setup[0].y) * Math.sin(setup[0].theta - Math.atan2(setup[0].y, setup[0].x))},
                {Math.cos(alpha + setup[1].theta), Math.sin(alpha + setup[1].theta), Math.hypot(setup[1].x, setup[1].y) * Math.sin(setup[1].theta - Math.atan2(setup[1].y, setup[1].x))},
                {Math.cos(alpha + setup[2].theta), Math.sin(alpha + setup[2].theta), Math.hypot(setup[2].x, setup[2].y) * Math.sin(setup[2].theta - Math.atan2(setup[2].y, setup[2].x))}
        };
        return A;
    }

}
