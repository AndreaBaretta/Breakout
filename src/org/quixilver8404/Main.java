package org.quixilver8404;

import org.quixilver8404.controller.Controller;
import org.quixilver8404.feedforward.*;
import org.quixilver8404.simulator.Display;
import org.quixilver8404.simulator.MecanumKinematics;
import org.quixilver8404.controller.PowerProfile;
import org.quixilver8404.util.Config;
import org.quixilver8404.util.Vector3;

import java.io.File;
import java.util.Arrays;
import java.util.List;

public class Main {

    public static void main(String[] args) {
        System.out.println("Howdy world");

        Display window1 = new Display(1000, 1000, 100);
        window1.init();

        final double v_max = 1.17749;
        final double a_max = 11.2;

        final double rX = 0.4572/2;
        final double rY = 0.4572/2;
        final double Tmax = 2.1;
        final double omegamax = 31.4;
        final double R = 37.5/1000;
        final double m = 20;
        final double J = MecanumKinematics.FindMomentOfInertia(0.5, 0.5, m);


        final MecanumKinematics kinematics = new MecanumKinematics(
                50, m, 0.5, 0.5,
                new Vector3(0,0,0), new Vector3(8.5* Config.INCHES_TO_METERS, 32.5*Config.INCHES_TO_METERS,Math.PI),
                window1,
                J, rX, rY, Tmax, R, omegamax);

        final PowerProfile powerProfile = new PowerProfile(m, R, J, omegamax, Tmax, rX, rY, false);
        final Controller controller = new Controller(Controller.computeK(m, R, J, omegamax, Tmax, rX, rY));

        double error_xy = 0;
        double error_alpha = 0;
        double t = 0;
        final double updateControllerEveryHz = 50;
        double counter = 0;
        double[] prevCorrection = new double[]{0,0,0};
//        double t1 = System.currentTimeMillis()/(double)1000;
//        double t2 = System.currentTimeMillis()/(double)1000;
        double t1 = (double)System.nanoTime()/(double)1e9;
        double t2 = (double)System.nanoTime()/(double)1e9;
        final double time_limit = 5;
        double maxAccel = 0;

        final List<ActionEventListener> actionEventListeners = Arrays.asList(new ActionEventListener[]{
                new ActionEventListener(1, new ActionFunction() {
                    @Override
                    public void run() {
                        System.out.println("1 executed");
                    }
                }),
                new ActionEventListener(2, new ActionFunction() {
                    @Override
                    public void run() {
                        System.out.println("2 executed");
                    }
                }),
                new ActionEventListener(3, new ActionFunction() {
                    @Override
                    public void run() {
                        System.out.println("3 executed");
                    }
                })
        });
        final Path path = new Path(new File("/home/andrea/Desktop/foxtrotFiles/three_stone_close_auton_blue.foxtrot2"), 0, actionEventListeners);

        double prev_s = 0;
        double prev_s_dot = 0;
        while (true) {
//            System.out.println("tan(pi/2): " + Math.tan(Math.PI/2));
//            System.out.println(Math.PI);
//            final double dt = 0.0001; //Preset time
//            double dt = t2 - t1;
            double dt = 0.001;

            final double s;
            final double s_dot;
            final double s_dot_dot;

//            t1 = t2;

            if (dt == 0) { //Only when running real-time simulation
                s = path.calcS(kinematics.getFieldPos().x, kinematics.getFieldPos().y);
                s_dot = 0;
                s_dot_dot = 0;
//                System.out.println("Set to 0: t2=" + t2 + "  t1=" + t1);
            } else {
                s = path.calcS(kinematics.getFieldPos().x, kinematics.getFieldPos().y);
                s_dot = (s - prev_s)/dt;
                s_dot_dot = path.calcAccelerationCorrection(s, s_dot);
//                System.out.println("Not 0");
            }

            t1 = t2;


            prev_s = s;

            final RobotState state = path.evaluate(s, s_dot, s_dot_dot);

//            System.out.println();

            final double[] correction;
            if (counter == updateControllerEveryHz) {
                correction = controller.correction(Vector3.subtractVector2(kinematics.getFieldPos(), state.pos),
                        Vector3.subtractVector(kinematics.getFieldVel(), state.vel));
                prevCorrection = correction;
                counter = 0;
            } else {
                correction = prevCorrection;
            }


            final double[] powerSettings = powerProfile.powerSetting(state.acc, state.vel, correction, kinematics.getFieldPos().theta);

            for (final double p : powerSettings) {
                if (Double.isNaN(p)) {
                    System.out.println("state.vel: " + state.vel.toString());
                    System.out.println("state.acc: " + state.acc.toString());
                    System.out.println("NaN power setting");
                    break;
                }
            }

            kinematics.update(powerSettings, dt);

//            for (int i = 0; i < path.segmentPoints.size(); i++) { //Draw segmentpoints
//                final SegmentPoint segmentPoint = path.segmentPoints.get(i);
//                final Vector3 pointPos = path.mainSegments.get(segmentPoint.anchorIndex).getPosition(segmentPoint.getS());
//                kinematics.ui.drawCircle(pointPos.x, pointPos.y, 0.05, 100, new double[]{255,255,0});
//            }

            kinematics.ui.drawCircle(state.pos.x, state.pos.y, 0.05, 100, new double[]{255,0,0});
            kinematics.ui.drawCircle(0, 0, 0.05, 100, new double[]{0,255,0});
            kinematics.ui.drawCompassPixel(state.pos.theta, 400, -400, 50);
            kinematics.ui.update();
            counter++;
            t+=dt;
            error_xy += Math.sqrt(Math.hypot(state.pos.x-kinematics.getFieldPos().x, state.pos.y-kinematics.getFieldPos().y))*dt;
            error_alpha += (state.pos.theta-kinematics.getFieldPos().theta)*dt;
            t2 = System.currentTimeMillis()/(double)1000;
//            if (counter >= 20000) {
//                kinematics.ui.terminate();
////                System.out.println("Average error in x-y: " + error_xy/t);
////                System.out.println("Average error in alpha: " + error_alpha/t);
//                break;
//            }
        }
    }
}