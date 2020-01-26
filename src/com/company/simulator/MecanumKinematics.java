package com.company.simulator;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.lwjgl.glfw.GLFW;
import org.lwjgl.opengl.GL11;

import java.util.Arrays;

public class MecanumKinematics {
    public final MecanumWheel[] wheels;
    public final double dt;
    public final double mass;
    public final double width;
    public final double length;
    public final double I;
    protected double[] fieldVel;
    protected double[] fieldPos;
    protected final Display ui;

    /**
     * Constructs the physics of the system.
     * @param wheels - The 4 wheels of the robot.
     * @param frequency - The frequency at which the robot operates, in hertz.
     * @param mass - The mass of the robot in kilograms.
     * @param width - The width of the robot in meters.
     * @param length - The length of the robot in meters.
     * @param startVelocity - The beginning velocity of the robot in meters per second and radians per second.
     * @param startPosition - The beginning position of the robot in meters and radians.
     * @param ui - The Display object that will draw the simulation.
     * */
    public MecanumKinematics(final MecanumWheel[] wheels, final double frequency, final double mass, final double width, final double length, final VectorXYAlpha startVelocity, final VectorXYAlpha startPosition, final Display ui) {
        this.wheels = wheels;
        this.dt = 1/frequency;
        this.mass = mass;
        this.width = width;
        this.length = length;
        I = findMomentOfInertia(width, length, mass);
        fieldVel = startVelocity.toArray();
        fieldPos = startPosition.toArray();
        this.ui = ui;
//        ui.init();
    }

    /**
     * Finds the moment of inertia of a rectangular surface.
     * @param w - Width of the surface in meters.
     * @param l - Length of the surface in meters.
     * @param m - Mass of the object in kilograms.
     * */
    protected double findMomentOfInertia(final double w, final double l, final double m) {
        final double sigma = m/(w*l);
        final double a = -w/2;
        final double b = w/2;
        final double c = -l/2;
        final double d = l/2;
        final double I = sigma*((1d/3d)*Math.pow(b,3)*d + (1d/3d)*b*Math.pow(d,3) - (1d/3d)*Math.pow(a,3)*d - (1d/3d)*a*Math.pow(d,3)
                -(1d/3d)*Math.pow(b,3)*c - (1d/3d)*b*Math.pow(c,3) + (1d/3d)*Math.pow(a,3)*c + (1d/3d)*a*Math.pow(c,3));
        return I;
    }

    /**
     * Updates the system.
     * @param newPowerSetting - The new power setting of the wheels, ranging from -1 to 1.
     * @param delta_t - The difference in time.
     * */
    public void update(final double[] newPowerSetting, final double delta_t) {
        if (newPowerSetting.length != wheels.length) {
            throw new IndexOutOfBoundsException("The number of power settings is not equivalent to that of wheels.");
        }
        double[] acceleration = new double[]{0,0,0};
        final double[] relativePos = CoordinateTransformations.toRelativeCoordinates(new ArrayRealVector(new double[]{fieldPos[0], fieldPos[1]}), fieldPos[2]).toArray();
        final double[] relativeVel = CoordinateTransformations.toRelativeVelocity(new ArrayRealVector(new double[]{relativePos[0], relativePos[1], fieldPos[2]}), new ArrayRealVector(fieldVel)).toArray();
        for (int i = 0; i < newPowerSetting.length; i++) {
            final double[] v_D = new double[]{relativeVel[0]-fieldVel[2]*wheels[i].Y, relativeVel[1]+fieldVel[2]*wheels[i].X};
            final double[] u_X = new double[]{1,0};
            final double[] u_Y = new double[]{0,1};
            final double v_W = -dot(v_D, u_X)*wheels[i].tangent - dot(v_D, u_Y);
            final double[] curForce = wheels[i].updateForce(newPowerSetting[i], v_W, delta_t);
            System.out.println("Velocity: " + Arrays.toString(fieldVel) + " Position" + Arrays.toString(fieldPos) + " v_W fct: " + wheels[i].motor.wheelVelocityFraction + " Power setpoint: " + newPowerSetting[i] + " Current Power: " + wheels[i].motor.currentPower + " Torque fct: " + wheels[i].motor.torqueFraction + " Force: " + Arrays.toString(curForce));
            final double[] fieldRelativeForce = CoordinateTransformations.toFieldCoordinates(new ArrayRealVector(curForce), fieldPos[2]).toArray();
            acceleration[0] += fieldRelativeForce[0]/mass;
            acceleration[1] += fieldRelativeForce[1]/mass;
            final double torque = wheels[i].X*curForce[1] - wheels[i].Y*curForce[0];
            acceleration[2] += torque/I;
        }
        System.out.println();
        fieldVel = vectorAddition(fieldVel, scalarMul(acceleration, delta_t));
        fieldPos = vectorAddition(fieldPos, scalarMul(fieldVel, delta_t));
        GL11.glClear(GL11.GL_COLOR_BUFFER_BIT);
//        GLFW.glfwSwapBuffers(ui.window);
        ui.setBackground(new double[]{255,255,255});
        ui.drawRobot(fieldPos[0], fieldPos[1], fieldPos[2],width, length, 100);
        ui.update();
    }


    /**
     * Updates the system with the preset frequency.
     * @param newPowerSetting - The new power setting of the wheels, ranging from -1 to 1.
     * */
    public void updatePresetFrequency(final double[] newPowerSetting) {
        update(newPowerSetting, dt);
    }

    /**
     * Performs a scalar multiplication on a vector.
     * @param a - Vector to be multiplied.
     * @param s - Scalar factor.
     * */
    public static double[] scalarMul(final double[] a, final double s) {
        final double[] result = new double[a.length];
        for (int i = 0; i < a.length; i++) {
            result[i] = s*a[i];
        }
        return result;
    }

    /**
     * Performs the dot product between two vectors.
     * */
    public static double dot(final double[] a1, final double[] a2) {
        if (a1.length != a2.length) {
            throw new IndexOutOfBoundsException("To perform a dot product, both arrays need to be of the same length.");
        }
        double sum = 0;
        for (int i = 0; i < a1.length; i++) {
            sum += a1[i]*a2[i];
        }
        return sum;
    }

    /**
     * Performs the addition between two vectors.
     * */
    public static double[] vectorAddition(final double[] a1, final double[] a2) {
        if (a1.length != a2.length) {
            throw new IndexOutOfBoundsException("To perform an addition, both arrays need to be of the same length.");
        }
        final double[] result = new double[3];
        for (int i = 0; i < a1.length; i++) {
            result[i] = a1[i]+a2[i];
        }
        return result;
    }

    /**
     * You really should be able to figure this one out by yourself.
     * */
    public VectorXYAlpha getFieldVel() { return VectorXYAlpha.fromArray(fieldVel); }

    /**
     * Level 2 of "Figuring it Out: Revenge of the Getters".
     * */
    public VectorXYAlpha getFieldPos() { return VectorXYAlpha.fromArray(fieldPos); }
}
