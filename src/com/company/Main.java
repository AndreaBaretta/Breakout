package com.company;

import com.company.feedforward.*;
import com.company.simulator.*;

import java.util.ArrayList;
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
                new Vector3(0,0,0), new Vector3(0, 1,0),
                window1,
                J, rX, rY, Tmax, R, omegamax);

        final PowerProfile powerProfile = new PowerProfile(m, R, J, omegamax, Tmax, rX, rY, false);
        final Controller controller = new Controller(Controller.computeK(m, R, J, omegamax, Tmax, rX, rY));

        final FeedForwardTest feedForwardTest = new FeedForwardTest();

        double error_xy = 0;
        double error_alpha = 0;
        double t = 0;
        final double updateControllerEveryHz = 300;
        double counter = 0;
        double[] prevCorrection = new double[]{0,0,0};
        double t1 = System.currentTimeMillis()/(double)1000;
        double t2 = System.currentTimeMillis()/(double)1000;
        final double time_limit = 5;
        double maxAccel = 0;

        final List<AnchorPoint> anchorPoints = new ArrayList<AnchorPoint>();
        anchorPoints.add(new AnchorPoint(0, 0, 0, AnchorPoint.Heading.CUSTOM, 3*Math.PI/2, 0, null, 0,
                1, new Point2D(0, 1), 0, null, new Point2D(1, 1), Config.MAX_VELOCITY, true, false));
        anchorPoints.add(new AnchorPoint(2, 4, 0, AnchorPoint.Heading.FRONT, 0, 1, new Point2D(2, 3), Math.PI,
                0, null, 0, new Point2D(1, 3), null, Config.MAX_VELOCITY, false, true));
        final Path path = new Path(anchorPoints);
        while (true) {
//            System.out.println("tan(pi/2): " + Math.tan(Math.PI/2));
//            System.out.println(Math.PI);
//            final double dt = 0.0001; //Preset time
            final double dt = t2 - t1; //Real-time simulation
            t1 = System.currentTimeMillis()/(double)1000;
//            final Vector3 pos = feedForwardTest.getPosition(t);
//            final Vector3 vel = feedForwardTest.getVelocity(t);
//            final Vector3 acc = feedForwardTest.getAcceleration(t);
            final Vector3 pos = path.evaluate(t, 0, 0).pos;

//            final double[] correction;
//            if (counter == updateControllerEveryHz) {
//                correction = controller.correction(Vector3.subtractVector(kinematics.getFieldPos(), pos),
//                        Vector3.subtractVector(kinematics.getFieldVel(), vel));
//                prevCorrection = correction;
//                counter = 0;
//            } else {
//                correction = prevCorrection;
//            }
//            final double[] powerSettings = powerProfile.powerSetting(acc, vel, correction, kinematics.getFieldPos().theta);
//            kinematics.update(powerSettings, dt);

//            System.out.println("Desired pos: " + pos.toString() + " actual pos: " + kinematics.getFieldPos() +
//                    " || Desired vel: " + vel.toString() + " actual vel: " + kinematics.getFieldVel() +
//                    " || Desired acc: " + acc.toString() + " actual acc: " + kinematics.getFieldAcc() +
//                    " || Power settings: " + Arrays.toString(powerSettings));

//            System.out.println(path.mainSegments.get(0).circleSegment1.theta1_);

            kinematics.ui.setBackground(new double[]{255,255,255});

            kinematics.ui.drawCircle(pos.x, pos.y, 0.05, 100, new double[]{255,0,0});
            kinematics.ui.drawCircle(0, 0, 0.05, 100, new double[]{0,255,0});
            kinematics.ui.drawCompassPixel(pos.theta, 400, -400, 50);

            kinematics.ui.update();
//            System.out.println("Current accel: " + Math.abs(kinematics.getFieldAcc().y));
//            if (Math.abs(kinematics.getFieldAcc().y) > maxAccel) {
//                maxAccel = Math.abs(kinematics.getFieldAcc().y);
//                System.out.println("maxAccel: " + maxAccel);
//            }
//            System.out.println(kinematics.getFieldAcc());
            counter++;
            t+=dt;
            error_xy += Math.sqrt(Math.hypot(pos.x-kinematics.getFieldPos().x, pos.y-kinematics.getFieldPos().y))*dt;
            error_alpha += (pos.theta-kinematics.getFieldPos().theta)*dt;
            t2 = System.currentTimeMillis()/(double)1000;
//            if (t >= time_limit) {
//                kinematics.ui.terminate();
//                System.out.println("Average error in x-y: " + error_xy/t);
//                System.out.println("Average error in alpha: " + error_alpha/t);
//                break;
//            }
        }
    }
}