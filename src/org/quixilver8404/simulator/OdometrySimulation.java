package org.quixilver8404.simulator;

import com.opencsv.CSVIterator;
import com.opencsv.CSVReader;
import com.opencsv.exceptions.CsvValidationException;
import org.quixilver8404.breakout.util.Config;
import org.quixilver8404.breakout.util.Vector3;

import java.io.File;
import java.io.FileReader;
import java.io.IOException;
import java.util.Arrays;

public class OdometrySimulation {

    public static final double ODOMETRY_ENCODER_M_PER_TICK = 0.035*Math.PI/8192 * 3.13055/3.105;
    public static final Vector3 ODOMETRY_1_POSITION = new Vector3(-0.1115,-0.0525,-Math.PI/2);
    public static final Vector3 ODOMETRY_2_POSITION = new Vector3(0.0065,0.185,Math.PI);
    public static final Vector3 ODOMETRY_3_POSITION = new Vector3(0.1115,0.0155,-Math.PI/2);
    public static final Vector3[] ODOMETRY_WHEEL_SETUP = new Vector3[]{ODOMETRY_1_POSITION, ODOMETRY_2_POSITION, ODOMETRY_3_POSITION};

    public static void main(final String[] args) throws IOException, CsvValidationException, InterruptedException {
        Display window1 = new Display(1000, 1000, 100);
        window1.init();
        final Config config = new Config(
                12, 12, 20.9345794393-10, -12,
                /*1.1775*/ 1,0.05, 2.1, 31.4, 10.7, 0.075/2, MecanumKinematics.FindMomentOfInertia(0.323, 0.445, 10.7),
                ((0.323/2) - 0.0375), ((0.445/2) - 0.05031), 0.95, 0.95, Math.PI/2.5, 0.17, 0.09, 10.7/4, 10.7/4, 10.7/4, 10.7/4
        );
        final MecanumKinematics kinematics = new MecanumKinematics(50, Config.MASS, 0.445, 0.323, new Vector3(0,0,0),
                new Vector3(0, 0,-Math.PI/2), window1, Config.J, Config.r_X, Config.r_Y, Config.T_MAX,
                Config.WHEEL_RADIUS, Config.OMEGA_MAX);


        final File file1 = new File("/home/andrea/Desktop/run-1.csv");
        final CSVIterator odoFile1 = new CSVIterator(new CSVReader(new FileReader(file1)));

        final Odometry odometry = new Odometry(ODOMETRY_WHEEL_SETUP, new Vector3(0,0, 0));

        long t1 = -1;
        double initialImu = Double.NaN;
        double initialImu = Double.NaN;
        odoFile1.next();
        while (odoFile1.hasNext()) {
            final String[] line = odoFile1.next();
            if (t1 == -1) {
                t1 = Long.parseLong(line[7]);
            }
            if (Double.isNaN(initialImu)) {
                initialImu = Double.parseDouble(line[6]);
            }
            final long t2 = Long.parseLong(line[7]);

            final double odo1 = Integer.parseInt(line[0]) * ODOMETRY_ENCODER_M_PER_TICK;
            final double odo2 = Integer.parseInt(line[1]) * ODOMETRY_ENCODER_M_PER_TICK;
            final double odo3 = Integer.parseInt(line[2]) * ODOMETRY_ENCODER_M_PER_TICK;
            odometry.update(new double[]{odo1, odo2, odo3}, (t2 - t1)/1000d, odometry.pos.theta);
            t1 = t2;

            System.out.println("Data: " + Arrays.toString(line));
            System.out.println("odoPos: " + odometry.pos.toString());

            kinematics.setFieldPos(odometry.pos);
            kinematics.ui.drawCircle(0, 0, 0.03, 100, new double[]{255,0,0});
//            kinematics.ui.drawCompassPixel(breakout.getLastDesiredPos().theta, 0, 0, 50);
            kinematics.ui.update();

            Thread.sleep(10);
        }
    }

}
