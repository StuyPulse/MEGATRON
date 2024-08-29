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
        {4000, 551},
        {4000, 573},
        {4000,546 },
        {4000, 566},
        {4000, 579},
        {4000, 458},
        {4000, 533},
        {3800, 561},
        {3800, 499},
        {3800, 576},
        {3800, 517},
        {3800, 497},
        {3800, 522},
        {3600, 456},
        {3600, 516},
        {3600, 465},
        {3600, 422},
        {3600, 542},
        {3400, 435},
        {3400, 467},
        {3400, 496},
        {3400, 461},
        {3400, 470},
        {3400, 448},
        {3400, 516},
        {3200, 433},
        {3200, 456},
        {3200, 447},
        {3200, 406},
        {3200, 424},
        {3200, 454},
        {3000, 414},
        {3000, 388},
        {3000, 389},
        {3000, 392},
        {3000, 405},
        {3000, 400},
        {3000, 430},
        {2600, 316},
        {2600, 354},
        {2600, 339},
        {2200, 288},
        {2200, 241},
        {2200, 268},
        {2200, 218},
        {2200, 210},
        {1500, 145},
        {1500, 145},
        {1500, 136},
        {1500, 184},
        {1500, 178},
    };
    // private static final double[][] RPMAndDistance = {
    //     {1000, 55.5},
    //     {1000, 42.5},
    //     {1000, 57},
    //     {1200, 73},
    //     {2900, 295},
    //     {2900, 265.5},
    //     {2900, 279},
    //     {2900, 289},
    //     {3000, 306},
    //     {3000, 317},
    //     {3000, 311.5},
    //     {3000, 324},
    //     {3000, 303},
    //     {3000, 300},
    //     {3000, 303.5},
    //     {3700, 437},
    //     {3700, 447.5},
    //     {3700, 419},
    //     {3700, 416},
    //     {3700, 400},
    //     {3700, 381.5},
    //     {3800, 410.5},
    //     {3800, 441.25},
    //     {3900, 457.5},
    //     {3900, 427},
    //     {4000, 476},
    //     {4000, 435},
    //     {4000, 439},
    // };

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
