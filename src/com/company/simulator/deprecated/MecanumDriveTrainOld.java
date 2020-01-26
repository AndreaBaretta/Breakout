package com.company.simulator.deprecated;

import com.company.simulator.VectorXYAlpha;

public class MecanumDriveTrainOld {
    final MecanumWheelOld[] mecanums;
    final VectorXYAlpha[] setup;
    final double mass;
    final MecanumKinematicsOld kinematics;
    final double k;
    final double dt;

    public MecanumDriveTrainOld(final MecanumWheelOld[] mecanums, final double mass, final double traction, final double width, final double height, final double hertzRate) {
        assert (mecanums.length == 4);
        this.mecanums = mecanums;
        this.mass = mass;
        setup = new VectorXYAlpha[]{
                new VectorXYAlpha(mecanums[0].X, mecanums[0].Y, mecanums[0].phi),
                new VectorXYAlpha(mecanums[1].X, mecanums[1].Y, mecanums[1].phi),
                new VectorXYAlpha(mecanums[2].X, mecanums[2].Y, mecanums[2].phi),
                new VectorXYAlpha(mecanums[3].X, mecanums[3].Y, mecanums[3].phi)
        };
        k = traction;
        kinematics = new MecanumKinematicsOld(setup, mass, k, width, height);
        dt = 1d/hertzRate;
    }

    public void iteratePresetFrequency(final double newPowerSetting) {
        for (MecanumWheelOld mecanumWheelOld : mecanums) {
            mecanumWheelOld.iterate(newPowerSetting, kinematics.v_Y, dt);
        }
        kinematics.update(new double[] {
                mecanums[0].getWheelVelocity(),
                mecanums[1].getWheelVelocity(),
                mecanums[2].getWheelVelocity(),
                mecanums[3].getWheelVelocity()
        });
    }

    public double[] getRobotCoords() { return kinematics.getXYAlphaField(); }

    public double[] getRobotVelocity() { return kinematics.getVelocityRobot(); }

    public double[] getFieldCoords() { return kinematics.getXYAlphaField(); }

    public double[] getFieldVelocity() { return kinematics.getVelocityField(); }
}
