package com.company;

import com.company.quixilver8404.skystone.opmode.test.FoxtrotTest;
import com.company.simulator.*;
import com.company.simulator.deprecated.FeedForwardTest;
import org.lwjgl.*;
import com.company.simulator.Motor.LowPassFilter;

public class Main {

    public Main() {

    }

    public static void run(final MecanumKinematics kinematics) {
        System.out.println("Hello LWJGL " + Version.getVersion() + "!");
//        window1.init();

        int count = 0;

        long t_new = System.nanoTime();
        long t_old = t_new;
        final FoxtrotTest foxtrotTest = new FoxtrotTest();
        double[] power = new double[]{-1,1,1,1};

        //for (int i = 0; i < 1000; i++) {
        while(/*window1.isRunning()*/ true) {
            t_new = System.nanoTime();
            final double dt = (t_new - t_old)/1e9d;
            t_old = t_new;

            assert dt > 0.0d;
            System.out.println("dt = " + dt);
//            t1 = System.currentTimeMillis();
//            try {
//                foxtrotTest.runOpMode(kinematics);
//            } catch (InterruptedException e) {
//                e.printStackTrace();
//            }
//            kinematics.updatePresetFrequency(power);
            kinematics.update(power, dt);
            count++;
            System.out.println("count = " + count);
//            if (count == 100) {
//                power = new double[]{0,0,0,0};
//            }
//            GL11.glClear(GL11.GL_COLOR_BUFFER_BIT);
//            GLFW.glfwSwapBuffers(window1.window);
//            try {
//                foxtrotTest.runOpMode();
//            } catch (InterruptedException e) {
//                e.printStackTrace();
//            }

//            window1.setBackground(new double[] {1,1,1});

//            window1.drawPolygon(new double[][] {
//                    {0+count,0},{100+count,0},{100+count,100},{0+count,100} }, new double[] {255,0,0});
//
//            window1.drawCircle(-100,-100, 50, 100, new double[] {0,255,0});
            final Vector3 pos = kinematics.getFieldPos();
            final Vector3 vel = kinematics.getFieldVel();
//            window1.drawRobot(pos.x,pos.y, pos.phi, kinematics.width, kinematics.length, 100);
//            window1.update();


        }



//        window1.terminate();
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

        Display window1 = new Display(1000, 1000);
        window1.init();

        final double rX = 0.4572/2;
        final double rY = 0.4572/2;
        final double Tmax = 2.1;
        final double omegamax = 31.4;
        final double R = 17.5/1000;
        final double J = 0.6967728;
        final double m = 13.6;
        final double[] rX_i = new double[]{rX, -rX, -rX, rX};
        final double[] rY_i = new double[]{rY, -rY, -rY, rY};


        final MecanumKinematics kinematics = new MecanumKinematics(
                50, m, 0.5, 0.5,

                new Vector3(0,0,-3), new Vector3(0,0,3.1415),

                window1,
                J, rX, rY, Tmax, R, omegamax);

        final PowerProfile powerProfile = new PowerProfile(m, R, J, omegamax, Tmax, rX, rY, false);
        final Controller controller = new Controller(Controller.computeK(m, R, J, omegamax, Tmax, rX, rY));

        final FeedForwardTest feedForwardTest = new FeedForwardTest();
//        final long t0 = System.currentTimeMillis();
        int counter = 0;
        double t = 0;
        final double dt = 0.005;
        while (true) {
//            run(kinematics);
//            counter+=1;
//            final long sys_t = System.currentTimeMillis();
//            final double t = (sys_t-t0)/1000d;

            final Vector3 pos = feedForwardTest.getPosition(t);
            final Vector3 vel = feedForwardTest.getVelocity(t);
            final Vector3 acc = feedForwardTest.getAcceleration(t);

            final double[] correction = controller.correction(Vector3.subtractVector(kinematics.getFieldPos(), pos), Vector3.subtractVector(kinematics.getFieldVel(), vel), kinematics.getFieldPos().theta);
            final double[] powerSettings = powerProfile.powerSetting(acc, vel, correction, kinematics.getFieldPos().theta);


//            System.out.println("Pos: Desired: " + pos.toString() + " Actual: " + kinematics.getFieldPos().toString() + " delta: " + Vector3.subtractVector(pos, kinematics.getFieldPos()).toString());
//            System.out.println("Vel: Desired: " + vel.toString() + " Actual: " + kinematics.getFieldVel().toString() + " delta: " + Vector3.subtractVector(vel, kinematics.getFieldVel()).toString());
//            System.out.println("t = " + t);

//            double[] finalPowerSetting = new double[4];
//            for (int i = 0; i < 4; i++) {
//               finalPowerSetting[i] = powerSettings[i] + correc-3*Math.pow(Math.cos(t),3) + 6*Math.cos(t)*Math.pow(Math.sin(t),2), -3*Math.pow(Math.sin(t),3) + 6*Math.sin(t)*Math.pow(Math.cos(t),2)tion[i];
//            }
//            System.out.println(Arrays.toString(powerSettings));
//            final double[] powerSettings = new double[] {1,-1,-1,1};

//            kinematics.update(powerSettings, dt);
            kinematics.update(powerSettings, dt);
//            System.out.println("Desired pos: " + pos.toString() + " actual pos: " + kinematics.getFieldPos() +
//                    " || Desired vel: " + vel.toString() + " actual vel: " + kinematics.getFieldVel() +
//                    " || Desired acc: " + acc.toString() + " actual acc: " + kinematics.getFieldAcc() +
//                    " || Power settings: " + Arrays.toString(powerSettings));
            kinematics.ui.drawCircle(pos.x, pos.y, 0.05, 100, new double[]{255,0,0},100);
            kinematics.ui.drawCircle(0, 0, 0.05, 100, new double[]{0,255,0},100);


            kinematics.ui.update();
            System.out.println();

            t+=dt;

        }
    }
}