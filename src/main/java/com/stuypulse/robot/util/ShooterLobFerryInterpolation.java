package com.stuypulse.robot.util;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;

/** Interpolation based on ferry angle of 50 degrees */
public class ShooterLobFerryInterpolation {

    public static void main(String[] args) {
        System.out.println(getRPM(450));
    }
    
    private static final InterpolatingDoubleTreeMap interpolatingDoubleTreeMap;

    // RPM, distance (inches)
    private static final double[][] RPMAndDistance = {
        // fake data im just putting here so it doesnt crash lol
        {3100,382},
        {3100,292},
        {3100,337},
        {3100,337},
        {3100,290},
        {3100,300},
        {3200,335},
        {3200,349},
        {3200,337},
        {3200,339},
        {3300,379},
        {3300,384},
        {3300,389},
        {3300,373},
        {3300,354},
        {3500,389},
        {3500,331},
        {2500,238},
        {2500,233},
        {2500,259},
        {2000,169},
        {2500,288},
        {2000,164},
        {2000,191},
        {1500,109},
        {1500,109},
        {1500,108},
        {1500,105},
        {1200,79},
        {1200,82},
        {1200,67},
        {1200,69},
        {1200,64},
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
