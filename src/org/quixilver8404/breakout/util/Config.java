package org.quixilver8404.breakout.util;

public class Config {
    public static double VOLTAGE_SCALE = 1;
    public static double MAX_ACCELERATION = 11.2;
    public static double MAX_DECELERATION = -11.2;
    public static double MAX_VELOCITY = 1.17749;
    public static double ACCELERATION_CORRECTION_STEP = 0.01;
    public static double T_MAX = 2.1;
    public static double OMEGA_MAX = 31.4;
    public static double MASS = 20;
    public static double WHEEL_RADIUS = 37.5/1000;
    public static double J = 0.8333333333333333;
    public static double r_X = 0.4572/2d;
    public static double r_Y = 0.4572/2d;
    public static double MAX_SAFE_VELOCITY = 0.95;
    public static double MAX_SAFE_ACCELERATION = 0.95;
    public static double ACCELERATION_CORRECTION = Math.PI/2.5;
    public final static double INCHES_TO_METERS = 0.0254;
    public static double P_STATIC = 0.2;
    public static double P_DYNAMIC = 0.1;
    public static double[] FRICTION_SCALAR_FACTOR_WHEEL = new double[]{1,1,1,1};
    public final static double P_static_physics = 0.17;
    public final static double P_dynamic_physics = 0.09;
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
    public final double P_static;
    public final double P_dynamic;
    public final double friction_scalar_factor_wheel_1;
    public final double friction_scalar_factor_wheel_2;
    public final double friction_scalar_factor_wheel_3;
    public final double friction_scalar_factor_wheel_4;
    protected double voltage_scale;
    public final double normal_voltage;

    public Config(final double normal_voltage, final double current_voltage, final double max_acceleration, final double max_deceleration, final double max_velocity,
                  final double acceleration_correction_step, final double t_max, final double omega_max, final double mass, final double wheel_radius, final double J, final double r_X,
                  final double r_Y, final double max_safe_velocity, final double max_safe_acceleration, final double acceleration_correction, final double P_static, final double P_dynamic,
                  final double mass_wheel_1, final double mass_wheel_2, final double mass_wheel_3, final double mass_wheel_4) {
        this.normal_voltage = normal_voltage;
        this.voltage_scale = current_voltage/normal_voltage;
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
        this.P_static = P_static;
        this.P_dynamic = P_dynamic;
        this.friction_scalar_factor_wheel_1 = mass_wheel_1/(mass/4);
        this.friction_scalar_factor_wheel_2 = mass_wheel_2/(mass/4);
        this.friction_scalar_factor_wheel_3 = mass_wheel_3/(mass/4);
        this.friction_scalar_factor_wheel_4 = mass_wheel_4/(mass/4);
    }

    public void set() {
        VOLTAGE_SCALE = voltage_scale;
        MAX_ACCELERATION = max_acceleration*voltage_scale;
        MAX_DECELERATION = max_deceleration*voltage_scale;
        MAX_VELOCITY = max_velocity*voltage_scale;
        ACCELERATION_CORRECTION_STEP = acceleration_correction_step;
        T_MAX = t_max*voltage_scale;
        OMEGA_MAX = omega_max*voltage_scale;
        MASS = mass;
        WHEEL_RADIUS = wheel_radius;
        J = inertia;
        r_X = rX;
        r_Y = rY;
        MAX_SAFE_VELOCITY = max_safe_velocity*voltage_scale;
        MAX_SAFE_ACCELERATION = max_safe_acceleration*voltage_scale;
        ACCELERATION_CORRECTION = acceleration_correction;
        P_STATIC = P_static/voltage_scale;
        P_DYNAMIC = P_dynamic/voltage_scale;
        FRICTION_SCALAR_FACTOR_WHEEL[0] = friction_scalar_factor_wheel_1;
        FRICTION_SCALAR_FACTOR_WHEEL[1] = friction_scalar_factor_wheel_2;
        FRICTION_SCALAR_FACTOR_WHEEL[2] = friction_scalar_factor_wheel_3;
        FRICTION_SCALAR_FACTOR_WHEEL[3] = friction_scalar_factor_wheel_4;
    }

    public void setVoltage(final double voltage) {
        voltage_scale = voltage/normal_voltage;
        VOLTAGE_SCALE = voltage_scale;
        set();
    }
}