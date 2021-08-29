package org.quixilver8404.breakout;

import org.quixilver8404.breakout.controller.Breakout;
import org.quixilver8404.breakout.simulator.Display;
import org.quixilver8404.breakout.simulator.MecanumKinematics;
import org.quixilver8404.breakout.util.Config;
import org.quixilver8404.breakout.util.Vector3;

import java.io.File;
import java.util.ArrayList;

public class BreakoutTest {

    public static void main(final String[] args) {
        Display window1 = new Display(1000, 1000, 100);
        window1.init();

        final Breakout breakout = new Breakout(new File("/home/andrea/Desktop/foxtrotFiles/power_shot_and_wobble_goals_blue_breakout2.foxtrot2"), 1, new ArrayList<>(),
                new Config(12, 12,11.2, -11.2, 1.17749, 0.01, 2.1, 31.4, 20,
                        37.5/1000, MecanumKinematics.FindMomentOfInertia(0.5, 0.5, 20), 0.4572/2, 0.4572/2, 0.95,
                        0.95, Math.PI/2.5, 0.2, 0.1,20d/4, 20d/4, 20d/4, 20d/4));

        final MecanumKinematics kinematics = new MecanumKinematics(50, Config.MASS, 0.5, 0.5, new Vector3(0,0,0),
                new Vector3(breakout.path.startX, breakout.path.startY,breakout.path.startHeading - Math.PI/2), window1, Config.J, Config.r_X, Config.r_Y, Config.T_MAX,
                Config.WHEEL_RADIUS, Config.OMEGA_MAX);

//        System.out.println(breakout == null);
//        System.out.println(breakout.path == null);
        while (true) {
            final double[] powerSettings = breakout.iterate(kinematics.getFieldPos(), kinematics.getFieldVel(), 0.0002);
            kinematics.update(powerSettings, 0.0002);
            kinematics.ui.drawCircle(breakout.getLastDesiredPos().x, breakout.getLastDesiredPos().y, 0.03, 100, new double[]{0,255,0});
            kinematics.ui.drawCircle(0, 0, 0.03, 100, new double[]{255,0,0});
            kinematics.ui.drawCompassPixel(breakout.getLastDesiredPos().theta, 0, 0, 50);
            kinematics.ui.update();
        }
    }
}
