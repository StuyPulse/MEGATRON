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
        {4000, 562},
        {4000, 538},
        {4000, 578},
        {4000, 498},
        {4000, 527},
        {4000, 499},
        {4000, 481},
        {4000, 611},
        {4000, 528},
        {4000, 365},
        {4000, 487},
        {4000, 548},
        {3800, 456},
        {3800, 457},
        {3800, 515},
        {3800, 563},
        {3800, 560},
        {3800, 495},
        {3600, 519},
        {3600, 505},
        {3600, 525},
        {3600, 513},
        {3600, 521},
        {3600, 513},
        {3600, 554},
        {3600, 495},
        {3400, 428},
        {3400, 415},
        {3400, 430},
        {3400, 428},
        {3400, 437},
        {3200, 439},
        {3200, 425},
        {3200, 455},
        {3200, 446},
        {3200, 447},
        {3200, 447},
        {3000, 413},
        // {3000, 428},
        // {3000, 418},
        {3000, 413},
        {3000, 369},
        {3000, 400},
        {3000, 342},
        // {3000, 426},
        {2600, 285},
        // {2600, 316},
        {2600, 302},
        {2600, 304},
        // {2600, 338},
        // {2600, 318},
        {2200, 258},
        // {2200, 307},
        {2200, 267},
        {2200, 267},
        // {2200, 306},
        {2200, 272},
        // {1500, 194},
        // {1500, 198},
        {1500, 132},
        {1500, 133},
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
