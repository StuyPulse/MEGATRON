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
