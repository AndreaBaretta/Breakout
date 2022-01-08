package org.quixilver8404.breakout;

import org.quixilver8404.breakout.controller.AutoPilot;
import org.quixilver8404.simulator.Display;
import org.quixilver8404.simulator.MecanumKinematics;
import org.quixilver8404.breakout.util.Config;
import org.quixilver8404.breakout.util.Vector3;

public class AutoPilotTest {

    public static void main(final String[] args) {
        Display window1 = new Display(1000, 1000, 100);
        window1.init();

        final AutoPilot autoPilot = new AutoPilot(
                new Config(12,12,18.512,-18.512,5.625,0.01,2.1,
                    31.4,12.1,(75d/2d)/1000d,
                        MecanumKinematics.FindMomentOfInertia(44d/100d, 44d/100d, 12.1),44d/200d - 0.05, 44d/200d - 0.073,
                    0.95,0.95,Math.PI/2.5, 0.17, 0.09, 12.1/4, 12.1/4, 12.1/4, 12.1/4), true
//                new Config(11.2, -11.2, 1.17749, 0.01, 2.1, 31.4, 20,
//                        37.5/1000, MecanumKinematics.FindMomentOfInertia(0.5, 0.5, 20), 0.4572/2, 0.4572/2, 0.95,
//                        0.95, Math.PI/2.5)
        );

        final Vector3 desiredPos = new Vector3(1, 1, 90*Math.PI/180);

        autoPilot.setDesiredPos(desiredPos);

        final MecanumKinematics kinematics = new MecanumKinematics(50, Config.MASS, 0.5, 0.5, new Vector3(0,0,0),
                new Vector3(0, 0,0), window1, Config.J, Config.r_X, Config.r_Y, Config.T_MAX,
                Config.WHEEL_RADIUS, Config.OMEGA_MAX);

        long t1 = System.currentTimeMillis();

        while (true) {
            final double[] powerSettings;
//            powerSettings = autoPilot.correction2(kinematics.getFieldPos(), kinematics.getFieldVel(),0.001);

            powerSettings = autoPilot.correction(kinematics.getFieldPos(), kinematics.getFieldVel());

//            System.out.println(Arrays.toString(powerSettings));
            System.out.println("Robot Velocity: " + kinematics.getFieldVel().toString());


            kinematics.update(powerSettings, 0.001);
            kinematics.ui.drawCircle(desiredPos.x, desiredPos.y, 0.03, 100, new double[]{0,255,0});
            kinematics.ui.drawCircle(0, 0, 0.03, 100, new double[]{255,0,0});
            kinematics.ui.drawCompassPixel(desiredPos.theta, 0, 0, 50);
            kinematics.ui.update();

            final long t2 = System.currentTimeMillis();
            System.out.println("Time for iteration = " + (t2 - t1) + "ms");
            t1 = t2;
            System.out.println();

//
//            autoPilot.controller.integral = Vector3.addVector(autoPilot.controller.integral, new Vector3(0.001,0.000,0.000));
//            final double[] dwArray = new double[] {
//                    0,0,0,0,0,0,
//                    autoPilot.controller.integral.x,
//                    autoPilot.controller.integral.y,
//                    autoPilot.controller.integral.theta
//            };
//            final ArrayRealVector dw = new ArrayRealVector(dwArray);
//            final double[] du = autoPilot.controller.K.operate(dw).toArray();
//            for (double p : du) {
//                if (p>=0.5) {
//                    System.out.println("Integral: " + autoPilot.controller.integral.x);
//                    System.exit(0);
//                }
//            }

        }
    }
}
