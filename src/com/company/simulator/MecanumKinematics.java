package com.company.simulator;

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

    /*
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

    /*
     * Finds the moment of inertia of a rectangular surface.
     * @param w - Width of the surface in meters.
     * @param l - Length of the surface in meters.
     * @param m - Mass of the object in kilograms.
     * */
//    protected double findMomentOfInertia(final double w, final double l, final double m) {
//        final double sigma = m/(w*l);
//        final double a = -w/2;
//        final double b = w/2;
//        final double c = -l/2;
//        final double d = l/2;
//        final double I = sigma*((1d/3d)*Math.pow(b,3)*d + (1d/3d)*b*Math.pow(d,3) - (1d/3d)*Math.pow(a,3)*d - (1d/3d)*a*Math.pow(d,3)
//                -(1d/3d)*Math.pow(b,3)*c - (1d/3d)*b*Math.pow(c,3) + (1d/3d)*Math.pow(a,3)*c + (1d/3d)*a*Math.pow(c,3));
//        System.out.println("Inertia: " + I);
//        return I;
//    }

    /**
     * Updates the system.
     * @param newPowerSetting - The new power setting of the wheels, ranging from -1 to 1.
     * @param delta_t - The difference in time.
     * */
    public void update(final double[] newPowerSetting, final double delta_t) {
//        if (newPowerSetting.length != 4) {
//            throw new IndexOutOfBoundsException("The number of power settings is not equivalent to that of wheels.");
//        }
//        double F_x = 0;
//        double F_y = 0;
//        double tau_alpha = 0;
//        for (int i = 0; i < 4; i++) {
//            final double phi = Math.pow(-1, i+1)*Math.PI/4;
//            System.out.println("phi: " + phi);
//            final double v_DX = -fieldVel.theta*rY[i] + fieldVel.x*Math.cos(fieldPos.theta) + fieldVel.y*Math.sin(fieldPos.theta);
//            final double v_DY = fieldVel.theta*rX[i] + -fieldVel.x*Math.sin(fieldPos.theta) + fieldVel.y*Math.cos(fieldPos.theta);
//            final double v_DP = Math.sin(phi)*v_DX + Math.cos(phi)*v_DY;
//            final double v_WP = -v_DP;
//            final double v_W = v_WP/Math.cos(phi);
//            final double T_m = Tmax*newPowerSetting[i] - (Tmax/(omegamax*R))*v_W;
//            final double T_d = -T_m;
//            final double F_Y = T_d/R;
//            final double F_X = F_Y*Math.tan(phi);
//
//            final double tau_alpha_i = rX[i]*F_Y - rY[i]*F_X;
//            final double F_xi = F_X*Math.cos(fieldPos.theta) - F_Y*Math.sin(fieldPos.theta);
//            final double F_yi = F_X*Math.sin(fieldPos.theta) + F_Y*Math.cos(fieldPos.theta);
//
//            F_x += F_xi;
//            F_y += F_yi;
//            tau_alpha += tau_alpha_i;




            /*   noise testing   */
//            final Random rand = new Random();
//            F_x += Math.pow(-1, rand.nextInt(1))*rand.nextDouble();
//            F_y += Math.pow(-1, rand.nextInt(1))*rand.nextDouble();
//            tau_alpha += Math.pow(-1, rand.nextInt(1))*rand.nextDouble();

//        }

        System.out.println("Actual alpha: " + fieldPos.theta);
        final double P_X = newPowerSetting[0]*Math.cos(fieldPos.theta) + newPowerSetting[1]*Math.sin(fieldPos.theta);
        final double P_Y = -newPowerSetting[0]*Math.sin(fieldPos.theta) + newPowerSetting[1]*Math.cos(fieldPos.theta);

        System.out.println("P_x P_y: " + Arrays.toString(new double[]{newPowerSetting[0], newPowerSetting[1]}));
        System.out.println("P_X P_Y: " + Arrays.toString(new double[]{P_X, P_Y}));
        System.out.println("P_alpha: " + newPowerSetting[2]);
//
//        /*----------------------------P_X P_Y -> P_i ----------------------------------------*/
//
        final double P_1 = P_Y - P_X + newPowerSetting[2];
        final double P_2 = P_Y + P_X - newPowerSetting[2];
        final double P_3 = P_Y - P_X - newPowerSetting[2];
        final double P_4 = P_Y + P_X + newPowerSetting[2];

//        P[1]:=P[Y]-P[X]+P[\[Alpha]]
//        P[2]:=P[Y]+P[X]-P[\[Alpha]]
//        P[3]:=P[Y]-P[X]-P[\[Alpha]]
//        P[4]:=P[Y]+P[X]+P[\[Alpha]]

        System.out.println(Arrays.toString(new double[] {P_1, P_2, P_3, P_4}));

        double noise_x = 0;
        double noise_y = 0;
        double noise_alpha = 0;

        final Random rand = new Random();
        noise_x += Math.pow(-1, rand.nextInt(1))*rand.nextDouble()*35;
        noise_y += Math.pow(-1, rand.nextInt(1))*rand.nextDouble()*35;
        noise_alpha += Math.pow(-1, rand.nextInt(1))*rand.nextDouble()*5;

        final double[] P = newPowerSetting;
//        final double R_omegamax = R*omegamax;
        final double R2_omegamax = Math.pow(R, 2)*omegamax;
        final double Tmax4_div_R2_omegamax = -4*Tmax/R2_omegamax;
        final double coefficient_Px = Tmax4_div_R2_omegamax * R * omegamax;
        final double coefficient_xdot = Tmax4_div_R2_omegamax;
        final double coefficient_Py = coefficient_Px;
        final double coefficient_ydot = coefficient_xdot;
        final double coefficient_Palpha = -4*Tmax*(rX+rY)*R*omegamax/(Math.pow(R,2)*omegamax);
        final double coefficient_alphadot = -4*Tmax*Math.pow((rX+rY),2)/(Math.pow(R,2)*omegamax);
//        final double F_x = -4*Tmax*( R_omegamax*P[0] - fieldVel.x )/R2_omegamax;
//        final double F_y = -4*Tmax*( R_omegamax*P[1] - fieldVel.y )/R2_omegamax;
//
//        final double Tmax_rXrY = Tmax*(rX[0] + rY[0]);
//        final double tau_alpha = -4*Tmax_rXrY*( R_omegamax*P[2] + ((rX[0] + rY[0])*fieldVel.theta) )/ R2_omegamax;

//        System.out.println("R2_omegamax: " + R2_omegamax);
//        System.out.println("Tmax4_div_R2_omegamax: " + Tmax4_div_R2_omegamax);
//        System.out.println("Tmax4_div_R2_omegamax * R * omegamax: " + Tmax4_div_R2_omegamax * R * omegamax);
//        System.out.println("coefficient_Px: " + coefficient_Px);
//        System.out.println("coefficient_xdot: " + coefficient_xdot);
//        System.out.println("coefficient_Py: " + coefficient_Py);
//        System.out.println("coefficient_ydot: " + coefficient_ydot);
//        System.out.println("coefficient_Palpha: " + coefficient_Palpha);

        final double cx_P1_3 = (Math.cos(fieldPos.theta) + Math.sin(fieldPos.theta))*Tmax/R;
        final double cx_P2_4 = (-Math.cos(fieldPos.theta) + Math.sin(fieldPos.theta))*Tmax/R;

        final double cy_P1_3 = (-Math.cos(fieldPos.theta) + Math.sin(fieldPos.theta))*Tmax/R;
        final double cy_P2_4 = (-Math.cos(fieldPos.theta) - Math.sin(fieldPos.theta))*Tmax/R;


        System.out.println("coefficient_x_P_1_3: " + cx_P1_3);
        System.out.println("coefficient_x_P_1_3: " + cx_P2_4);
        System.out.println("Tmax/R: " + Tmax/R);

        final double c_P_alpha = ((rX+rY)*Tmax/R);

        final double F_x = cx_P1_3*P_1 + cx_P2_4*P_2 + cx_P1_3*P_3 + cx_P2_4*P_4 + coefficient_xdot*fieldVel.x + noise_x;
        final double F_y = cy_P1_3*P_1 + cy_P2_4*P_2 + cy_P1_3*P_3 + cy_P2_4*P_4 + coefficient_xdot*fieldVel.y + noise_y;
        final double tau = -c_P_alpha*P_1 + c_P_alpha*P_2 + c_P_alpha*P_3 - c_P_alpha*P_4 + coefficient_alphadot*fieldVel.theta + noise_alpha;

        System.out.println("Forces: " + Arrays.toString(new double[]{F_x, F_y, tau}));
        System.out.println("c_P_alpha: " + c_P_alpha);
        System.out.println("coefficient_alphadot: " + coefficient_alphadot);

        System.out.println("(rX*Tmax/R): " + (rX*Tmax/R));

//        final double xdotdot = coefficient_Px*P[0] + coefficient_xdot*fieldVel.x + noise_x;
//        final double ydotdot = coefficient_Py*P[1] + coefficient_ydot*fieldVel.y + noise_y;
//        final double alphadotdot = coefficient_Palpha*P[2] + coefficient_alphadot*fieldVel.theta + noise_alpha;

//        System.out.println("Vel: " + fieldVel.toString());
//        System.out.println("xdotdot: " + xdotdot + " ydotdot: " + ydotdot + " alphadotdot: " + alphadotdot);


//        System.out.println("F_x: " + F_x + " F_y: " + F_y + " tau_alpha: " + tau_alpha);
        fieldAcc = new Vector3(F_x/mass, F_y/mass, tau/J);
        System.out.println("ax: " + fieldAcc.x + " ay: " + fieldAcc.y + " aa: " + fieldAcc.theta);
        fieldVel = Vector3.addVector(fieldVel, fieldAcc.scalarMultiply(delta_t));
        System.out.println("vx: " + fieldVel.x + " vy: " + fieldVel.y + " va: " + fieldVel.theta);
        fieldPos = Vector3.addVector(fieldPos, fieldVel.scalarMultiply(delta_t));
        System.out.println("px: " + fieldPos.x + " py: " + fieldPos.y + " pa: " + fieldPos.theta);

        GL11.glClear(GL11.GL_COLOR_BUFFER_BIT);
        ui.setBackground(new double[]{255,255,255});
        ui.drawRobot(fieldPos.x, fieldPos.y, fieldPos.theta, width, length, 100);
//        ui.update();
    }


    /**
     * Updates the system with the preset frequency.
     * @param newPowerSetting - The new power setting of the wheels, ranging from -1 to 1.
     * */
    public void updatePresetFrequency(final double[] newPowerSetting) {
        update(newPowerSetting, dt);
    }

//    /**
//     * Performs a scalar multiplication on a vector.
//     * @param a - Vector to be multiplied.
//     * @param s - Scalar factor.
//     * */
//    public static double[] scalarMul(final double[] a, final double s) {
//        final double[] result = new double[a.length];
//        for (int i = 0; i < a.length; i++) {
//            result[i] = s*a[i];
//        }
//        return result;
//    }
//
//    /**
//     * Performs the dot product between two vectors.
//     * */
//    public static double dot(final double[] a1, final double[] a2) {
//        if (a1.length != a2.length) {
//            throw new IndexOutOfBoundsException("To perform a dot product, both arrays need to be of the same length.");
//        }
//        double sum = 0;
//        for (int i = 0; i < a1.length; i++) {
//            sum += a1[i]*a2[i];
//        }
//        return sum;
//    }
//
//    /**
//     * Performs the addition between two vectors.
//     * */
//    public static double[] vectorAddition(final double[] a1, final double[] a2) {
//        if (a1.length != a2.length) {
//            throw new IndexOutOfBoundsException("To perform an addition, both arrays need to be of the same length.");
//        }
//        final double[] result = new double[3];
//        for (int i = 0; i < a1.length; i++) {
//            result[i] = a1[i]+a2[i];
//        }
//        return result;
//    }

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
