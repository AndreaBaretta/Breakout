package org.quixilver8404.breakout;

import org.quixilver8404.breakout.controller.Breakout;
import org.quixilver8404.simulator.Display;
import org.quixilver8404.simulator.MecanumKinematics;
import org.quixilver8404.breakout.util.Config;
import org.quixilver8404.breakout.util.Vector3;

import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;

public class BreakoutTest {

    public static void main(final String[] args) {
        Display window1 = new Display(1000, 1000, 100);
        window1.init();

//        final File file = new File("/home/andrea/Desktop/test.foxtrot2");
//        final File file = new File("/home/andrea/git/ftc8404/freight-frenzy/TeamCode/src/main/res/raw/auton_blue_carousel_first.foxtrot2");
        final File file = new File("/home/andrea/git/ftc8404/freight-frenzy/TeamCode/src/main/res/raw/auton_blue_no_carousel.foxtrot2");

//        final Config config = new Config(12, 12,11.2, -11.2, 1.17749, 0.01, 2.1, 31.4, 20,
//                37.5/1000, MecanumKinematics.FindMomentOfInertia(0.5, 0.5, 20), 0.4572/2, 0.4572/2, 0.95,
//                0.95, Math.PI/2.5, 0.2, 0.1,20d/4, 20d/4, 20d/4, 20d/4);
        final Config config = new Config(
                12, 12, 20.9345794393-10, -12,
                /*1.1775*/ 1,0.05, 2.1, 31.4, 10.7, 0.075/2, MecanumKinematics.FindMomentOfInertia(0.323, 0.445, 10.7),
                ((0.323/2) - 0.0375), ((0.445/2) - 0.05031), 0.95, 0.95, Math.PI/2.5, 0.17, 0.09, 10.7/4, 10.7/4, 10.7/4, 10.7/4
        );

        final Breakout breakout = new Breakout(file, 1, new ArrayList<>(), config);

        final MecanumKinematics kinematics = new MecanumKinematics(50, Config.MASS, 0.445, 0.323, new Vector3(0,0,0),
                new Vector3(breakout.path.startX, breakout.path.startY,breakout.path.startHeading - Math.PI/2), window1, Config.J, Config.r_X, Config.r_Y, Config.T_MAX,
                Config.WHEEL_RADIUS, Config.OMEGA_MAX);

//        System.out.println(breakout == null);
//        System.out.println(breakout.path == null);
        final double dt = 0.0005;
        while (true) {
            final double[] powerSettings = breakout.iterate(kinematics.getFieldPos(), kinematics.getFieldVel(), dt);
//            System.out.println("power settings: " + Arrays.toString(powerSettings));
            kinematics.update(powerSettings, dt);
//            System.out.println("Actual acceleration: " + kinematics.getFieldAcc().toString());
            kinematics.ui.drawCircle(breakout.getLastDesiredPos().x, breakout.getLastDesiredPos().y, 0.03, 100, new double[]{0,255,0});
            kinematics.ui.drawCircle(0, 0, 0.03, 100, new double[]{255,0,0});
            kinematics.ui.drawCompassPixel(breakout.getLastDesiredPos().theta, 0, 0, 50);
            kinematics.ui.update();
        }
    }
}
