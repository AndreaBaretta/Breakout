package com.company.simulator.deprecated;

public class MotorProfile {

    final double m;
    final double R;
    final double J;
    final double omegamax;
    final double Tmax;
    final boolean normalize;

    public MotorProfile(final double m, final double R, final double J, final double omegamax,
                        final double Tmax, final boolean normalize) {
        this.m = m;
        this.R = R;
        this.J = J;
        this.omegamax = omegamax;
        this.Tmax = Tmax;
        this.normalize = normalize;
    }

    public double[] powerSetting(final double[] desiredAccel, final double[] desiredVel, final double[] desiredPos) {
        final double mR2omegamax = m*(Math.pow(R, 2)*omegamax;
        final double RTmaxomegamax = R*Tmax*omegamax;
    }
}
