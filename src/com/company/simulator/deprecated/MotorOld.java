package com.company.simulator.deprecated;

public class MotorOld {
    protected final double mu_fs;
    protected final double mu_delta;
    protected final double startVel;
    protected final LowPassFilter filter;
    protected double v_D;
    protected double v_R;

    public MotorOld(final double mu_fs, final double mu_delta, final double startWheelVelocity) {
        this.mu_fs = mu_fs;
        this.mu_delta = mu_delta;
        filter = new LowPassFilter(20, 0);
        v_R = startWheelVelocity;
        startVel = startWheelVelocity;
    }

    public void update(final double powerSetting, final double chassisVelocity, final double dt) {
        final double P = filter.iterate(dt, powerSetting);
        v_D = chassisVelocity;

        final double mu_P = mu_fs/(1+mu_delta);
        final double mu_Vd = mu_delta/(1+mu_delta);

        v_R = mu_P * P + mu_Vd * v_D;
    }

    public LowPassFilter getFilter() { return filter;}

    public double getWheelVelocity() { return v_R; }


    public class LowPassFilter {
        public final double pole;
        protected double dxdt;
        protected double dx;
        protected double x;
        protected double u;

        public double get_pole() { return pole;}

        public double get_x() { return x;}
        public double get_dxdt() { return dxdt; }
        public double get_u() { return u;}

        public LowPassFilter(final double radiansPerSec, final double startState) {
            pole = radiansPerSec;
            x = startState;
        }

        public double iterate(final double dt, final double input) {
            u = input;
            dxdt = -pole*x + u;
            dx = dxdt*dt;
            x += dx;
            return pole*x;
        }
    }

    public MotorOld clone() {
        return new MotorOld(mu_fs, mu_delta, startVel);
    }
}
