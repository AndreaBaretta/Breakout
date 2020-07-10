package com.company.simulator;

import org.apache.commons.math3.linear.ArrayRealVector;
import org.apache.commons.math3.linear.RealVector;
import org.lwjgl.opengl.GL11;

import java.util.Arrays;
import java.util.Random;

public class MecanumKinematics {
    public final double dt;
    public final double mass;
    public final double width;
    public final double length;
    public final double J;
    protected Vector3 fieldAcc;
    protected Vector3 fieldVel;
    protected Vector3 fieldPos;
    public final Display ui;
    final double rX;
    final double rY;
    final double Tmax;
    final double R;
    final double omegamax;

    /**
     * Constructs the physics of the system.
     * @param frequency - The preset frequency at which the physics will update, in hz
     * @param mass - The mass of the robot, kg
     * @param width - Width of the robot, meters
     * @param length - Length of the robot, meters
     * @param startVelocity - Starting velocity of robot, in meters/s and radians/s
     * @param startPosition - Starting velocity of robot, in meters and radians
     * @param ui - OpenGL-based window to show the physics
     * @param J -Moment of inertia of robot
     * @param rX - The distance between the wheels and the center of the robot along the X-axis
     * @param rY - The distance between the wheels and the center of the robot along the X-axis
     * @param Tmax - The stall torque of the motors
     * @param R - The radius of the wheels
     * @param omegamax - The maximum angular velocity of the motors
     * */
    public MecanumKinematics(final double frequency, final double mass, final double width, final double length,
                             final Vector3 startVelocity, final Vector3 startPosition, final Display ui, final double J,
                             final double rX, final double rY, final double Tmax, final double R, final double omegamax) {
        this.dt = 1/frequency;
        this.mass = mass;
        this.width = width;
        this.length = length;
        this.J = J;
        fieldVel = startVelocity;
        fieldPos = startPosition;
        this.ui = ui;
        this.rX = rX;
        this.rY = rY;
        this.Tmax = Tmax;
        this.R = R;
        this.omegamax = omegamax;
//        ui.init();
    }

    /**
     * Finds the moment of inertia of an object with a rectangular base.
     * @param w - Width of the surface in meters.
     * @param l - Length of the surface in meters.
     * @param m - Mass of the object in kilograms.
     * */
    public static double FindMomentOfInertia(final double w, final double l, final double m) {
        final double sigma = m/(w*l);
        final double I = sigma*(1d/12d)*(Math.pow(w,3)*l + w*Math.pow(l,3));
        System.out.println("Inertia: " + I);
        return I;
    }

    /**
     * Updates the system.
     * @param newPowerSetting - The new power setting of the wheels, ranging from -1 to 1.
     * @param delta_t - The difference in time.
     * */
    public void update(final double[] newPowerSetting, final double delta_t) {


        final double P_1_ = newPowerSetting[0];
        final double P_2_ = newPowerSetting[1];
        final double P_3_ = newPowerSetting[2];
        final double P_4_ = newPowerSetting[3];

        final double P_1, P_2, P_3, P_4;


        if (Math.abs(P_1_) > 1) {
            P_1 = Math.abs(P_1_) / P_1_;
        } else {
            P_1 = P_1_;
        }
        if (Math.abs(P_2_) > 1) {
            P_2 = Math.abs(P_2_) / P_2_;
        } else {
            P_2 = P_2_;
        }
        if (Math.abs(P_3_) > 1) {
            P_3 = Math.abs(P_3_) / P_3_;
        } else {
            P_3 = P_3_;
        }
        if (Math.abs(P_4_) > 1) {
            P_4 = Math.abs(P_4_) / P_4_;
        } else {
            P_4 = P_4_;
        }

        double noise_x = 0;
        double noise_y = 0;
        double noise_alpha = 0;

        final Random rand = new Random();
//        noise_x += Math.pow(-1, rand.nextInt(1))*rand.nextDouble()*35;
//        noise_y += Math.pow(-1, rand.nextInt(1))*rand.nextDouble()*35;
//        noise_alpha += Math.pow(-1, rand.nextInt(1))*rand.nextDouble()*5;

        final double[] P = newPowerSetting;
        final double R2_omegamax = Math.pow(R, 2)*omegamax;
        final double Tmax4_div_R2_omegamax = -4*Tmax/R2_omegamax;
        final double coefficient_xdot = Tmax4_div_R2_omegamax;
        final double coefficient_ydot = coefficient_xdot;
        final double coefficient_alphadot = -4*Tmax*Math.pow((rX+rY),2)/(Math.pow(R,2)*omegamax);

        final double cx_P1_3 = (Math.cos(fieldPos.theta) + Math.sin(fieldPos.theta))*Tmax/R;
        final double cx_P2_4 = (-Math.cos(fieldPos.theta) + Math.sin(fieldPos.theta))*Tmax/R;

        final double cy_P1_3 = (-Math.cos(fieldPos.theta) + Math.sin(fieldPos.theta))*Tmax/R;
        final double cy_P2_4 = (-Math.cos(fieldPos.theta) - Math.sin(fieldPos.theta))*Tmax/R;

        final double c_P_alpha = ((rX+rY)*Tmax/R);

        final double F_x = cx_P1_3*P_1 + cx_P2_4*P_2 + cx_P1_3*P_3 + cx_P2_4*P_4 + coefficient_xdot*fieldVel.x + noise_x;
        final double F_y = cy_P1_3*P_1 + cy_P2_4*P_2 + cy_P1_3*P_3 + cy_P2_4*P_4 + coefficient_ydot*fieldVel.y + noise_y;
        final double tau = -c_P_alpha*P_1 + c_P_alpha*P_2 + c_P_alpha*P_3 - c_P_alpha*P_4 + coefficient_alphadot*fieldVel.theta + noise_alpha;

        fieldAcc = new Vector3(F_x/mass, F_y/mass, tau/J);
        fieldVel = Vector3.addVector(fieldVel, fieldAcc.scalarMultiply(delta_t));
        fieldPos = Vector3.addVector(fieldPos, fieldVel.scalarMultiply(delta_t));

        GL11.glClear(GL11.GL_COLOR_BUFFER_BIT);
        ui.setBackground(new double[]{255,255,255});
        ui.drawRobot(fieldPos.x, fieldPos.y, fieldPos.theta, width, length);
//        ui.update();
    }


    /**
     * Updates the system with the preset frequency.
     * @param newPowerSetting - The new power setting of the wheels, ranging from -1 to 1.
     * */
    public void updatePresetFrequency(final double[] newPowerSetting) {
        update(newPowerSetting, dt);
    }

    /**
    * Returns the acceleration of the robot.
    * */
    public Vector3 getFieldAcc() { return fieldAcc; }

    /**
     * You really should be able to figure this one out by yourself.
     * */
    public Vector3 getFieldVel() { return fieldVel; }

    /**
     * Level 2 of "Figuring it Out: Revenge of the Getters".
     * */
    public Vector3 getFieldPos() { return fieldPos; }
}
