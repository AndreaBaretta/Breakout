package org.quixilver8404.breakout.util;

public class Config {
    public static double MAX_ACCELERATION = 11.2;
    public static double MAX_DECELERATION = -11.2;
    public static double MAX_VELOCITY = 1.17749;
    public static double ACCELERATION_CORRECTION_STEP = 0.01;
    public static double T_MAX = 2.1;
    public static double OMEGA_MAX = 31.4;
    public static double MASS = 20;
    public static double WHEEL_RADIUS = 37.5/1000;
    public static double J = 0.8333333333333333;
    public static double r_X = 0.4572/2;
    public static double r_Y = 0.4572/2;
    public static double MAX_SAFE_VELOCITY = 0.95;
    public static double MAX_SAFE_ACCELERATION = 0.95;
    public static double ACCELERATION_CORRECTION = Math.PI/2.5;
    public final static double INCHES_TO_METERS = 0.0254;
    public final static double P_static = 0.35;
    public final static double P_dynamic = 0.1;
    public final static double P_static_physics = 0.30;
    public final static double P_dynamic_physics = 0.1;
//    public final static double P_static = 0;
//    public final static double P_dynamic = 0;
//    public final static double P_static_physics = 0;
//    public final static double P_dynamic_physics = 0;

    public final double max_acceleration;
    public final double max_deceleration;
    public final double max_velocity;
    public final double acceleration_correction_step;
    public final double t_max;
    public final double omega_max;
    public final double mass;
    public final double wheel_radius;
    public final double inertia;
    public final double rX;
    public final double rY;
    public final double max_safe_velocity;
    public final double max_safe_acceleration;
    public final double acceleration_correction;

    public Config(final double max_acceleration, final double max_deceleration, final double max_velocity, final double acceleration_correction_step,
                  final double t_max, final double omega_max, final double mass, final double wheel_radius, final double J, final double r_X, final double r_Y,
                  final double max_safe_velocity, final double max_safe_acceleration, final double acceleration_correction) {
        this.max_acceleration = max_acceleration;
        this.max_deceleration = max_deceleration;
        this.max_velocity = max_velocity;
        this.acceleration_correction_step = acceleration_correction_step;
        this.t_max = t_max;
        this.omega_max = omega_max;
        this.mass = mass;
        this.wheel_radius = wheel_radius;
        this.inertia = J;
        this.rX = r_X;
        this.rY = r_Y;
        this.max_safe_velocity = max_safe_velocity;
        this.max_safe_acceleration = max_safe_acceleration;
        this.acceleration_correction = acceleration_correction;
    }

    public void set() {
        MAX_ACCELERATION = max_acceleration;
        MAX_DECELERATION = max_deceleration;
        MAX_VELOCITY = max_velocity;
        ACCELERATION_CORRECTION_STEP = acceleration_correction_step;
        T_MAX = t_max;
        OMEGA_MAX = omega_max;
        MASS = mass;
        WHEEL_RADIUS = wheel_radius;
        J = inertia;
        r_X = rX;
        r_Y = rY;
        MAX_SAFE_VELOCITY = max_safe_velocity;
        MAX_SAFE_ACCELERATION = max_safe_acceleration;
        ACCELERATION_CORRECTION = acceleration_correction;
    }
}