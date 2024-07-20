package com.stuypulse.robot.util;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/** Interpolation based on ferry angle of -50 degrees */
public class ShooterLowFerryInterpolation {

    public static void main(String[] args) {
        System.out.println(getRPM(450));
    }
    
    private static final InterpolatingDoubleTreeMap interpolatingDoubleTreeMap;

    // RPM, distance (inches)
    private static final double[][] RPMAndDistance = {
        {1000, 55.5},
        {1000, 42.5},
        {1000, 57},
        {1200, 73},
        {2900, 295},
        {2900, 265.5},
        {2900, 279},
        {2900, 289},
        {3000, 306},
        {3000, 317},
        {3000, 311.5},
        {3000, 324},
        {3000, 303},
        {3000, 300},
        {3000, 303.5},
        {3700, 437},
        {3700, 447.5},
        {3700, 419},
        {3700, 416},
        {3700, 400},
        {3700, 381.5},
        {3800, 410.5},
        {3800, 441.25},
        {3900, 457.5},
        {3900, 427},
        {4000, 476},
        {4000, 435},
        {4000, 439},
    };

    static {
        interpolatingDoubleTreeMap = new InterpolatingDoubleTreeMap();
        for (double[] data: RPMAndDistance) {
            interpolatingDoubleTreeMap.put(data[1], data[0]);
        }
    }

    public static double getRPM(double distanceInInches) {
        return interpolatingDoubleTreeMap.get(distanceInInches);
    }
}
