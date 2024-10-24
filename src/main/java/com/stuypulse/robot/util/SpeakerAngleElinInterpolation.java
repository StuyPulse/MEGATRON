package com.stuypulse.robot.util;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import edu.wpi.first.math.util.Units;

/** Huge thanks to Elin for doing all the math */ 
public class SpeakerAngleElinInterpolation {

    private static final InterpolatingDoubleTreeMap interpolatingDoubleTreeMap;

    // distance (angle), angle (radians)
    private static final double[][] distanceAndAngle = {
        {45.1854245,-0.57751},
        {47.5, -0.60548},
        {50, -0.63458},
        {52.5, -0.66255},
        {55, -0.68943},
        {57.5,-0.71524},
        {60, -0.74002},
        {62.5, -0.76382},
        {65, -0.78668},
        {67.5, -0.80862},
        {70, -0.82969},
        {72.5, -0.84993},
        {75, -0.86938},
        {77.5, -0.88806},
        {80, -0.90602},
        {82.5, -0.92328},
        {85, -0.93989},
        {87.5, -0.95587},
        {90, -0.97125},
        {92.5, -0.98606},
        {95, -1.00032},
        {97.5, -1.01407},
        {100, -1.02732},
        {102.5, -1.0401},
        {105, -1.05243},
        {107.5, -1.06434},
        {110, -1.07583},
        {112.5, -1.08693},
        {115, -1.09767},
        {117.5, -1.10804},
        {120, -1.11808},
        {122.5, -1.12779},
        {125, -1.13719},
        {127.5, -1.14629},
        {130, -1.15512},
        {132.5, -1.16366},
        {135, -1.17195},
        {137.5, -1.17999},
        {140, -1.18779},
        {142.5, -1.19536},
        {145, -1.20271},
        {147.5, -1.20986},
        {150, -1.21679},
        {152.5, -1.22354},
        {155, -1.2301},
        {157.5, -1.23647},
        {160, -1.24268},
        {162.5, -1.24872},
        {165, -1.2546},
        {167.5, -1.26033},
        {170, -1.26591},
        {172.5, -1.27134},
        {175, -1.27664},
        {177.5, -1.28181},
        {180, -1.28685},
    };

    static {
        interpolatingDoubleTreeMap = new InterpolatingDoubleTreeMap();
        for (double[] data: distanceAndAngle) {
            interpolatingDoubleTreeMap.put(data[0], data[1]);
        }
    }

    public static double getAngleInRadians(double distanceInInches) {
        return interpolatingDoubleTreeMap.get(distanceInInches);
    }

    public static double getAngleInDegrees(double distanceInInches) {
        return Units.radiansToDegrees(getAngleInRadians(distanceInInches));
    }
}
