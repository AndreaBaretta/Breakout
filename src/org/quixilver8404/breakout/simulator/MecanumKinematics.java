package org.quixilver8404.breakout.simulator;

import org.lwjgl.opengl.GL11;
import org.lwjgl.system.CallbackI;
import org.quixilver8404.breakout.util.Config;
import org.quixilver8404.breakout.util.Vector3;

import java.util.ArrayList;
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
    public static double  FindMomentOfInertia(final double w, final double l, final double m) {
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

//        System.out.println("Received power settings: " + Arrays.toString(newPowerSetting));

        final double P_1_ = newPowerSetting[0];
        final double P_2_ = newPowerSetting[1];
        final double P_3_ = newPowerSetting[2];
        final double P_4_ = newPowerSetting[3];

        final double P_1, P_2, P_3, P_4;

        final double P_static = Config.P_static_physics;
        final double P_dynamic = Config.P_dynamic_physics;

        final double vel_1 = velWheel(rX, rY, -Math.PI/4, R);
        final double vel_2 = velWheel(-rX, rY, Math.PI/4, R);
        final double vel_3 = velWheel(-rX, -rY, -Math.PI/4, R);
        final double vel_4 = velWheel(rX, -rY, Math.PI/4, R);

//        P_1 = P_1_;
//        P_2 = P_2_;
//        P_3 = P_3_;
//        P_4 = P_4_;

        int threshold = 0;

        final boolean[] w_static = new boolean[4];

        if (Math.abs(vel_1) <= 1e-3 && Math.abs(P_1_) < P_static) { //Check if P_dynamic causes change in sign of velocity of wheel
//            System.out.println("1 did not overcome static");
            P_1 = 0;
            w_static[0] = true;
            threshold += 1;
        } else {
//            System.out.println("1 overcame static");
            w_static[0] = false;
            P_1 = Math.min(Math.max(P_1_, -1),1) - Math.signum(vel_1)*P_dynamic;
        }
        if (Math.abs(vel_2) <= 1e-3 && Math.abs(P_2_) < P_static) {
//            System.out.println("2 did not overcome static");
            w_static[1] = true;

            P_2 = 0;
            threshold += 1;
        } else {
//            System.out.println("2 overcame static");
            w_static[1] = false;

            P_2 = Math.min(Math.max(P_2_, -1),1) - Math.signum(vel_2)*P_dynamic;
        }
        if (Math.abs(vel_3) <= 1e-3 && Math.abs(P_3_) < P_static) {
//            System.out.println("3 did not overcome static");
            w_static[2] = true;

            P_3 = 0;
            threshold += 1;
        } else {
            w_static[2] = false;

//            System.out.println("3 overcame static");
            P_3 = Math.min(Math.max(P_3_, -1),1) - Math.signum(vel_3)*P_dynamic;
        }
        if (Math.abs(vel_4) <= 1e-3 && Math.abs(P_4_) < P_static) {
//            System.out.println("4 did not overcome static");
            w_static[3] = true;

            P_4 = 0;
            threshold += 1;
        } else {
            w_static[2] = false;
//            System.out.println("4 overcame static");
            P_4 = Math.min(Math.max(P_4_, -1),1) - Math.signum(vel_4)*P_dynamic;
        }

//        System.out.println("Final power 1: " + P_1 + ", velocity of wheel: " + vel_1 + ", initial power: " + P_1_);
//        System.out.println("Final power 2: " + P_2 + ", velocity of wheel: " + vel_2 + ", initial power: " + P_2_);
//        System.out.println("Final power 3: " + P_3 + ", velocity of wheel: " + vel_3 + ", initial power: " + P_3_);
//        System.out.println("Final power 4: " + P_4 + ", velocity of wheel: " + vel_4 + ", initial power: " + P_4_);

        double noise_x = 0;
        double noise_y = 0;
        double noise_alpha = 0;

        final Random rand = new Random();
//        noise_x += Math.pow(-1, rand.nextInt(2))*rand.nextDouble()*35;
//        noise_y += Math.pow(-1, rand.nextInt(2))*rand.nextDouble()*35;
//        noise_alpha += Math.pow(-1, rand.nextInt(2))*rand.nextDouble()*5;
//        System.out.println("noise_alpha: " + noise_alpha + " noise_x: " + noise_x + " noise_y: " + noise_y + " random_int: " + Math.pow(-1, rand.nextInt(2)));

//        final double[] P = newPowerSetting;
//        final double R2_omegamax = Math.pow(R, 2)*omegamax;
//        final double Tmax4_div_R2_omegamax = -4*Tmax/R2_omegamax;
//        final double coefficient_xdot = Tmax4_div_R2_omegamax;
//        final double coefficient_ydot = coefficient_xdot;
//        final double coefficient_alphadot = -4*Tmax*Math.pow((rX+rY),2)/(Math.pow(R,2)*omegamax);
//
//        final double cx_P1_3 = (Math.cos(fieldPos.theta) + Math.sin(fieldPos.theta))*Tmax/R;
//        final double cx_P2_4 = (-Math.cos(fieldPos.theta) + Math.sin(fieldPos.theta))*Tmax/R;
//
//        final double cy_P1_3 = (-Math.cos(fieldPos.theta) + Math.sin(fieldPos.theta))*Tmax/R;
//        final double cy_P2_4 = (-Math.cos(fieldPos.theta) - Math.sin(fieldPos.theta))*Tmax/R;
//
//        final double c_P_alpha = ((rX+rY)*Tmax/R);
//
//        final double F_x = cx_P1_3*P_1 + cx_P2_4*P_2 + cx_P1_3*P_3 + cx_P2_4*P_4 + coefficient_xdot*fieldVel.x + noise_x;
//        final double F_y = cy_P1_3*P_1 + cy_P2_4*P_2 + cy_P1_3*P_3 + cy_P2_4*P_4 + coefficient_ydot*fieldVel.y + noise_y;
//        final double tau = -c_P_alpha*P_1 + c_P_alpha*P_2 + c_P_alpha*P_3 - c_P_alpha*P_4 + coefficient_alphadot*fieldVel.theta + noise_alpha;


        final double cos_a = Math.cos(fieldPos.theta);
        final double sin_a = Math.sin(fieldPos.theta);

        final double[] FX = new double[] {
                Tmax * (R * P_1 * omegamax - (cos_a + sin_a)*getFieldVel().x + (cos_a - sin_a)*getFieldVel().y + (rX + rY)*fieldVel.theta)/(Math.pow(R,2) * omegamax),
                -Tmax * (R * P_2 * omegamax + (cos_a - sin_a)*getFieldVel().x + (cos_a + sin_a)*getFieldVel().y - (rX + rY)*fieldVel.theta)/(Math.pow(R,2) * omegamax),
                Tmax * (R * P_3 * omegamax - (cos_a + sin_a)*getFieldVel().x + (cos_a - sin_a)*getFieldVel().y - (rX + rY)*fieldVel.theta)/(Math.pow(R,2) * omegamax),
                -Tmax * (R * P_4 * omegamax + (cos_a - sin_a)*getFieldVel().x + (cos_a + sin_a)*getFieldVel().y + (rX + rY)*fieldVel.theta)/(Math.pow(R,2) * omegamax)
        };

        final double[] FY = new double[] {
                -Tmax * (R * P_1 * omegamax - (cos_a + sin_a)*getFieldVel().x + (cos_a - sin_a)*getFieldVel().y + (rX + rY)*fieldVel.theta)/(Math.pow(R,2) * omegamax),
                -Tmax * (R * P_2 * omegamax + (cos_a - sin_a)*getFieldVel().x + (cos_a + sin_a)*getFieldVel().y - (rX + rY)*fieldVel.theta)/(Math.pow(R,2) * omegamax),
                Tmax * (-R * P_3 * omegamax + (cos_a + sin_a)*getFieldVel().x + (-cos_a + sin_a)*getFieldVel().y + (rX + rY)*fieldVel.theta)/(Math.pow(R,2) * omegamax),
                -Tmax * (R * P_4 * omegamax + (cos_a - sin_a)*getFieldVel().x + (cos_a + sin_a)*getFieldVel().y + (rX + rY)*fieldVel.theta)/(Math.pow(R,2) * omegamax)
        };

        final double[] Fx = new double[4];
        final double[] Fy = new double[4];
        for (int i = 0; i < 4; i++) {
            Fx[i] = cos_a*FX[i]-sin_a*FY[i];
            Fy[i] = sin_a*FX[i]+cos_a*FY[i];
        }

        final double[] tau = new double[] {
                rX*FY[0]-rY*FX[0],
                -rX*FY[1]-rY*FX[1],
                -rX*FY[2]+rY*FX[2],
                rX*FY[3]+rY*FX[3],
        };

//        System.out.println("FY: " + Arrays.toString(FY));
//        System.out.println("Fy: " + Arrays.toString(Fy));
//
//        final double[] Fx = new double[]{
//                w_static[0] ? 0 : (R*P_1*omegamax + (-cos_a - sin_a)*fieldVel.x + (cos_a - sin_a)*fieldVel.y + (rX + rY)*fieldVel.theta)*(cos_a + sin_a)*Tmax/(Math.pow(R, 2)*omegamax),
//                w_static[1] ? 0 : (R*P_2*omegamax + (cos_a - sin_a)*fieldVel.x + (cos_a + sin_a)*fieldVel.y - (rX + rY)*fieldVel.theta)*(sin_a - cos_a)*Tmax/(Math.pow(R, 2)*omegamax),
//                w_static[2] ? 0 : (R*P_3*omegamax + (-cos_a - sin_a)*fieldVel.x + (cos_a - sin_a)*fieldVel.y - (rX + rY)*fieldVel.theta)*(sin_a + cos_a)*Tmax/(Math.pow(R, 2)*omegamax),
//                w_static[3] ? 0 : (R*P_4*omegamax + (cos_a - sin_a)*fieldVel.x + (cos_a + sin_a)*fieldVel.y + (rX + rY)*fieldVel.theta)*(sin_a - cos_a)*Tmax/(Math.pow(R, 2)*omegamax)
//        };
//
//        final double[] Fy = new double[]{
//                w_static[0] ? 0 : (R*P_1*omegamax + (-cos_a - sin_a)*fieldVel.x + (cos_a - sin_a)*fieldVel.y + (rX + rY)*fieldVel.theta)*(sin_a - cos_a)*Tmax/(Math.pow(R, 2)*omegamax),
//                w_static[1] ? 0 : (R*P_2*omegamax + (cos_a - sin_a)*fieldVel.x + (cos_a + sin_a)*fieldVel.y - (rX + rY)*fieldVel.theta)*(-sin_a - cos_a)*Tmax/(Math.pow(R, 2)*omegamax),
//                w_static[2] ? 0 : (R*P_3*omegamax + (-cos_a - sin_a)*fieldVel.x + (cos_a - sin_a)*fieldVel.y - (rX + rY)*fieldVel.theta)*(sin_a - cos_a)*Tmax/(Math.pow(R, 2)*omegamax),
//                w_static[3] ? 0 : (R*P_4*omegamax + (cos_a - sin_a)*fieldVel.x + (cos_a + sin_a)*fieldVel.y + (rX + rY)*fieldVel.theta)*(-sin_a - cos_a)*Tmax/(Math.pow(R, 2)*omegamax)
//        };
//
//        final double[] tau = new double[]{
//                w_static[0] ? 0 : (R*P_1*omegamax + (-cos_a - sin_a)*fieldVel.x + (cos_a - sin_a)*fieldVel.y + (rX + rY)*fieldVel.theta)*(-Tmax*(rX+rY)/(Math.pow(R,2)*omegamax)),
//                w_static[1] ? 0 : (R*P_2*omegamax + (cos_a - sin_a)*fieldVel.x + (cos_a + sin_a)*fieldVel.y - (rX + rY)*fieldVel.theta)*(Tmax*(rX+rY)/(Math.pow(R,2)*omegamax)),
//                w_static[2] ? 0 : (R*P_3*omegamax + (-cos_a - sin_a)*fieldVel.x + (cos_a - sin_a)*fieldVel.y - (rX + rY)*fieldVel.theta)*(Tmax*(rX+rY)/(Math.pow(R,2)*omegamax)),
//                w_static[3] ? 0 : (R*P_4*omegamax + (cos_a - sin_a)*fieldVel.x + (cos_a + sin_a)*fieldVel.y + (rX + rY)*fieldVel.theta)*(-Tmax*(rX+rY)/(Math.pow(R,2)*omegamax))
//        };
//
        double Fx_total = 0;
        for (double Fx_i : Fx) { Fx_total += Fx_i; }
        double Fy_total = 0;
        for (double Fy_i : Fy) { Fy_total += Fy_i; }
        double tau_total = 0;
        for (double tau_i : tau) { tau_total += tau_i; }

//        System.out.println("F_x_test=" + F_x_test + ", F_x=" + F_x);
//        System.out.println("F_y_test=" + F_y_test + ", F_y=" + F_y);
//        System.out.println("tau_test=" + tau_test + ", tau=" + tau);


        if (threshold >= 5) {
            fieldAcc = new Vector3(0,0,0);
            fieldVel = new Vector3(0,0,0);
        } else {
            fieldAcc = new Vector3(Fx_total / mass, Fy_total / mass, tau_total / J);
            fieldVel = Vector3.addVector(fieldVel, fieldAcc.scalarMultiply(delta_t));
            fieldPos = Vector3.addVector(fieldPos, fieldVel.scalarMultiply(delta_t));
        }

//        System.out.println(fieldVel.toString());

        GL11.glClear(GL11.GL_COLOR_BUFFER_BIT);
        ui.setBackground(new double[]{255,255,255});
        ui.drawRobot(fieldPos.x, fieldPos.y, fieldPos.theta+Math.PI/2, width, length);
//        ui.update();
//        System.out.println();
    }

    protected double velWheel(final double rX, final double rY, final double phi, final double r) {
        final Vector3 vel = getFieldVel();
        final double alpha = getFieldPos().theta;
        final double sin_a = Math.sin(alpha);
        final double cos_a = Math.cos(alpha);
        final double vdX = -vel.theta*rY + (vel.x*cos_a + vel.y*sin_a);
        final double vdY = vel.theta*rX + (-vel.x*sin_a + vel.y*cos_a);
        final double vw = -vdY - vdX*Math.tan(phi);
        return vw/r;
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
