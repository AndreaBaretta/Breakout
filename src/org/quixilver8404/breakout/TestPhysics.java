package org.quixilver8404.breakout;

import org.quixilver8404.simulator.Display;
import org.quixilver8404.simulator.MecanumKinematics;
import org.quixilver8404.breakout.util.Config;
import org.quixilver8404.breakout.util.Vector3;

public class TestPhysics {

    public static void main(final String[] args) {

        final Display window1 = new Display(1000,1000,100);
        final MecanumKinematics kinematics = new MecanumKinematics(50, Config.MASS, 0.5, 0.5, new Vector3(0,0,0),
                new Vector3(0,0,0), window1, Config.J, Config.r_X, Config.r_Y, Config.T_MAX,
                Config.WHEEL_RADIUS, Config.OMEGA_MAX);

        while (true) {


            kinematics.update(new double[]{1,-1,1,-1}, 0.0005);
            System.out.println("Field position: " + kinematics.getFieldPos());
        }
    }
}
