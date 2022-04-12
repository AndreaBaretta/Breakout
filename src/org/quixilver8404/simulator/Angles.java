package org.quixilver8404.simulator;

 class Angles {

    /**
     * Standardize the angle to between 0 and 2pi.
     * @param angle - degrees
     * @return - radians
     */
    public static double toStandardFull(final double angle) {
        final double temp = angle % (2*Math.PI);
        if (temp < 0) {
            return temp + 2*Math.PI;
        }
        return temp;
    }

    /**
     * Standardize the angle to between -pi and pi.
     * @param angle - degrees
     * @return - reduced version of angle and in radians
     */
    public static double toStandardHalf(final double angle) {
        // reduce the angle
        double temp =  angle % (2*Math.PI);

        // force it to be the positive remainder, so that 0 <= angle < 2PI
        temp = (temp + (2*Math.PI)) % (2*Math.PI);

        // force into the minimum absolute value residue class, so that -PI < angle <= PI
        if (temp > Math.PI) {
            temp -= 2*Math.PI;
        }

        return temp;
    }
}
