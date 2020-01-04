package com.company.simulator;

import org.apache.commons.math3.linear.ArrayRealVector;

public class MecanumWheel {
    public final Motor motor;
    public final double radius;
    public final double X;
    public final double Y;
    public final double phi;
    public final double m;
    public final double[] u_i;
    public final double tangent;

    /**
     * Wrapper class for the mecanum wheel.
     * @param motor - Motor attached to the wheel.
     * @param radius - Radius of the wheel in meters.
     * @param X - Position of the wheel relative to the center of the robot along the X axis, in meters.
     * @param Y - Position of the wheel relative to the center of the robot along the Y axis, in meters.
     * @param angle - Angle of the rollers of mecanum wheels, assuming wheel is perpendicular to X axis, in radians.
     * @param mass - Mass of the robot, kilograms.
     * @param unitVector - Unit vector along rollers.
     * */
    public MecanumWheel(final Motor motor, final double radius, final double X, final double Y, final double angle, final double mass, final double[] unitVector) {
        this.motor = motor;
        this.radius = radius;
        this.X = X;
        this.Y = Y;
        phi = angle;
        m = mass;
        u_i = unitVector;
        tangent = Math.tan(phi);
    }

    /**
     * Calculate the force applied by the wheel to the rest of the robot.
     * @param newPowerSetting - New power setting of the motor, from -1 to 1, proportional to voltage.
     * @param wheelVelocity - The angular velocity of the wheel, radians per second.
     * @param dt - Difference in time, in seconds.
     * */
    public double[] updateForce(final double newPowerSetting, final double wheelVelocity, final double dt) {
        motor.update(newPowerSetting, -wheelVelocity/radius, dt);
        final double torque = motor.torque;
        final double F_y = -torque/radius;
        final double F_x = tangent*F_y;
        return new double[] {F_x,F_y};
    }

    /**
     * Calculates the unit vector along u_P at a certain heading.
     * @param alpha - Angle of robot, in radians.
     * */
    public double[] getU_i(final double alpha) {
        return CoordinateTransformations.toFieldCoordinates(new ArrayRealVector(u_i), alpha).toArray();
    }
}
