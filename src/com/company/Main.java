package com.company;

import com.company.simulator.*;
import com.company.simulator.FeedForwardTest;

public class Main {

    public Main() {

    }

    public static void main(String[] args) {
        System.out.println("Howdy world");


//        final Motor motor = new Motor(2.1, 31.4, new LowPassFilter(20, 0));
//        final double phi = Math.PI/4;
//        final double m = 20;
//        final double[][] u_Pi = new double[][] {
//                {-Math.sin(phi), Math.cos(phi)},
//                {Math.sin(phi), Math.cos(phi)},
//                {-Math.sin(phi), Math.cos(phi)},
//                {Math.sin(phi), Math.cos(phi)}
//        };
//        final MecanumWheel wheel = new MecanumWheel(motor.clone(), 0.1, 1, 1, phi, m, u_Pi[0]);
//        final double r = 37.5d/1000d;
//        final double X = 0.4572/2d;
//        final double Y = 0.4572/2d;
//        final MecanumWheel[] wheels = new MecanumWheel[] {
//                new MecanumWheel(motor.clone(), r, X, Y, -phi, m, u_Pi[0]),
//                new MecanumWheel(motor.clone(), r, -X, Y, phi, m, u_Pi[1]),
//                new MecanumWheel(motor.clone(), r, -X, -Y, -phi, m, u_Pi[2]),
//                new MecanumWheel(motor.clone(), r, X, -Y, phi, m, u_Pi[3])
//        };

        Display window1 = new Display(1000, 1000, 50);
        window1.init();

        final double rX = 0.4572/2;
        final double rY = 0.4572/2;
        final double Tmax = 2.1;
        final double omegamax = 31.4;
        final double R = 37.5/1000;
        final double J = 0.141667;
        final double m = 13.6;

        final MecanumKinematics kinematics = new MecanumKinematics(
                50, m, 0.5, 0.5,
                new Vector3(0,0,0), new Vector3(10, 10,20),
                window1,
                J, rX, rY, Tmax, R, omegamax);

        final PowerProfile powerProfile = new PowerProfile(m, R, J, omegamax, Tmax, rX, rY, false);
        final Controller controller = new Controller(Controller.computeK(m, R, J, omegamax, Tmax, rX, rY));

        final FeedForwardTest feedForwardTest = new FeedForwardTest();
        double t = 0;
//        final double dt = 0.0001;
        double dt = 0;
        final double updateControllerEveryHz = 20;
        double counter = 0;
        double[] prevCorrection = new double[]{0,0,0};
        double t1 = System.currentTimeMillis()/(double)1000;
        double t2 = System.currentTimeMillis()/(double)1000;
        while (true) {
            dt = t2 - t1;
//            System.out.println(dt_realtime);
            t1 = System.currentTimeMillis()/(double)1000;
            final Vector3 pos = feedForwardTest.getPosition(t);
            final Vector3 vel = feedForwardTest.getVelocity(t);
            final Vector3 acc = feedForwardTest.getAcceleration(t);

            final double[] correction;
            if (counter == updateControllerEveryHz) {
                correction = controller.correction(Vector3.subtractVector(kinematics.getFieldPos(), pos),
                        Vector3.subtractVector(kinematics.getFieldVel(), vel), kinematics.getFieldPos().theta);
                prevCorrection = correction;
                counter = 0;
            } else {
                correction = prevCorrection;
            }
            final double[] powerSettings = powerProfile.powerSetting(acc, vel, correction, kinematics.getFieldPos().theta);
            kinematics.update(powerSettings, dt);
//            System.out.println("Desired pos: " + pos.toString() + " actual pos: " + kinematics.getFieldPos() +
//                    " || Desired vel: " + vel.toString() + " actual vel: " + kinematics.getFieldVel() +
//                    " || Desired acc: " + acc.toString() + " actual acc: " + kinematics.getFieldAcc() +
//                    " || Power settings: " + Arrays.toString(powerSettings));
            kinematics.ui.drawCircle(pos.x, pos.y, 0.05, 100, new double[]{255,0,0});
            kinematics.ui.drawCircle(0, 0, 0.05, 100, new double[]{0,255,0});
            kinematics.ui.drawCompassPixel(pos.theta, 400, -400, 50);

            kinematics.ui.update();
//            System.out.println();
            System.out.println(kinematics.getFieldVel());
            counter++;
            t+=dt;
            t2 = System.currentTimeMillis()/(double)1000;
        }
    }
}