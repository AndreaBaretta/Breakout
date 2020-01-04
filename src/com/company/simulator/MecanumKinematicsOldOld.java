package com.company.simulator;

public class MecanumKinematicsOldOld {
    protected final VectorXYAlpha[] setup;
    protected final double m;
    protected final double I;
    protected final double k;
    protected double[] v_Di_P;
    protected double[][] u_P_i;
    protected VectorXYAlpha d2_XYAlpha_dt2;
    protected VectorXYAlpha d_XYAlpha_dt;
    protected VectorXYAlpha XYAlpha;
    protected final MotorOld[] motorOlds;

    /**
     * Class for creating the kinematics system which will control the simulation.
     * @param setup - The positions for the position of the four wheels relative to the center of the robot. (m, radian)
     * @param mass - Mass of robot (kg)
     * @param inertia - Moment of inertia of the robot. (kg * m^2)
     * @throws IndexOutOfBoundsException - Simulation only works with 4 wheels.
     * */
    public MecanumKinematicsOldOld(final VectorXYAlpha[] setup, final double mass, final double inertia, final double slip) {
        if (setup.length != 4) {
            throw new IndexOutOfBoundsException("Only 4 wheels allowed.");
        }
        this.setup = setup;
        m = mass;
        I = inertia;
        k = slip;
        u_P_i = new double[][] {
                {-Math.sin(setup[0].phi), Math.cos(setup[0].phi)},
                {Math.sin(setup[1].phi), Math.cos(setup[1].phi)},
                {-Math.sin(setup[2].phi), Math.cos(setup[2].phi)},
                {Math.sin(setup[3].phi), Math.cos(setup[3].phi)}
        };
        d_XYAlpha_dt = new VectorXYAlpha(0,0,0);
        XYAlpha = new VectorXYAlpha(0,0,0);
        motorOlds = new MotorOld[] {
                new MotorOld(1160,640, 0),
                new MotorOld(1160,640, 0),
                new MotorOld(1160,640, 0),
                new MotorOld(1160,640, 0)
        };
    }

    protected void mecanumForceCalculation() {
        final VectorXYAlpha vel = d_XYAlpha_dt;
        for (int i = 0; i < setup.length; i++) {
            final VectorXYAlpha wheel_i = setup[i];
            v_Di_P[i] = Math.pow(-1, i) * wheel_i.sin * (vel.x - vel.phi * wheel_i.D * wheel_i.sin) + wheel_i.cos * (vel.y + vel.phi * wheel_i.D * wheel_i.cos);
        }
    }

    protected void updateAccel(final double[] v_Ri_P) {
        d2_XYAlpha_dt2 = new VectorXYAlpha(0,0,0); //reset accel
        for (int i = 0; i < 4; i++) {
            final double[] F_i = {k*(v_Ri_P[i]-v_Di_P[i]*u_P_i[i][0]), k*(v_Ri_P[i]-v_Di_P[i]*u_P_i[i][1])};
            d2_XYAlpha_dt2 = VectorXYAlpha.AddVector(d2_XYAlpha_dt2, new VectorXYAlpha(
                    Math.pow(-1, i) * (k / m) * (v_Ri_P[i] - v_Di_P[i]) * setup[i].sin,
                    (k / m) * (v_Ri_P[i] - v_Di_P[i]) * setup[i].cos,
                    (1/I)*(setup[i].x*F_i[1] - setup[i].y*F_i[0])
            ));
        }
    }

    /**
     * Method to update the state of the robot.
     * @param powerSetting - The setting of the individual wheels' power, normalized from -1 to 1 where the extremes are the maximum of the free-spinning wheels.
     * @param dt - Amount of time that passed.
     * */
    public void update(final double[] powerSetting, final double dt) {
        final double[] wheelVelocity = new double[4];
        for (int i = 0; i < 4; i++) {
            motorOlds[i].update(powerSetting[i], v_Di_P[i], dt);
            wheelVelocity[i] = motorOlds[i].getWheelVelocity();
        }
        final double[] v_Ri_P = new double[4];
        for (int i = 1; i < 4; i++) {
            v_Ri_P[i] = dot( new double[] {0,wheelVelocity[i]}, u_P_i[i]);
        }
        mecanumForceCalculation();
        updateAccel(v_Ri_P);
        d_XYAlpha_dt = VectorXYAlpha.AddVector(d_XYAlpha_dt, d2_XYAlpha_dt2.scalarMultiply(dt));
        XYAlpha = VectorXYAlpha.AddVector(XYAlpha, d_XYAlpha_dt.scalarMultiply(dt));
    }

    protected double dot(final double[] a1, final double[] a2) {
        if (a1.length != a2.length) {
            throw new IndexOutOfBoundsException("You idiot, the two arrays need to be the same length");
        }
        double sum = 0;
        for (int i = 0; i < a1.length-1; i++) {
            sum += a1[i] * a2[i];
        }
        return sum;
    }

    public VectorXYAlpha getRelativeAcceleration() { return  d2_XYAlpha_dt2; }

    public VectorXYAlpha getRelativeVelocity() { return d_XYAlpha_dt; }

    public VectorXYAlpha getRelativePosition() { return XYAlpha; }

}