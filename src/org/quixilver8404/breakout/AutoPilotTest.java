package org.quixilver8404.breakout;

import org.quixilver8404.breakout.controller.AutoPilot;
import org.quixilver8404.breakout.controller.Breakout;
import org.quixilver8404.breakout.simulator.Display;
import org.quixilver8404.breakout.simulator.MecanumKinematics;
import org.quixilver8404.breakout.util.Config;
import org.quixilver8404.breakout.util.Vector3;

import java.io.File;
import java.util.ArrayList;

public class AutoPilotTest {

    public static void main(final String[] args) {
        Display window1 = new Display(1000, 1000, 100);
        window1.init();

        final AutoPilot autoPilot = new AutoPilot(
                new Config(18.512,-18.512,5.625,0.01,2.1,
                    31.4,12.1,(75d/2d)/1000,
                        MecanumKinematics.FindMomentOfInertia(44d/100d, 44d/100d, 12.1),44d/200d - 0.05, 44d/200d - 0.073,
                    0.95,0.95,Math.PI/2.5)
//                new Config(11.2, -11.2, 1.17749, 0.01, 2.1, 31.4, 20,
//                        37.5/1000, MecanumKinematics.FindMomentOfInertia(0.5, 0.5, 20), 0.4572/2, 0.4572/2, 0.95,
//                        0.95, Math.PI/2.5)
        );

        autoPilot.setDesiredPos(new Vector3(0, 0, Math.PI+0.1));

        final MecanumKinematics kinematics = new MecanumKinematics(50, Config.MASS, 0.5, 0.5, new Vector3(0,0,0),
                new Vector3(0, 0,0), window1, Config.J, Config.r_X, Config.r_Y, Config.T_MAX,
                Config.WHEEL_RADIUS, Config.OMEGA_MAX);

        while (true) {
            final double[] powerSettings = autoPilot.correction(kinematics.getFieldPos(), kinematics.getFieldVel());
            kinematics.update(powerSettings, 0.001);
            kinematics.ui.update();
        }
    }
}