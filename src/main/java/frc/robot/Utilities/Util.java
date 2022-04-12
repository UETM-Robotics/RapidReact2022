package frc.robot.Utilities;

import java.util.List;

/**
 * Contains basic functions that are used often.
 */
public class Util {

    public static final double kEpsilon = 1e-12;
    /** Prevent this class from being instantiated. */
    private Util() {
    }

    /**
     * Limits the given input to the given magnitude.
     */
    public static double limit(double v, double maxMagnitude) {
        return limit(v, -maxMagnitude, maxMagnitude);
    }

    public static double limit(double v, double min, double max) {
        return Math.min(max, Math.max(min, v));
    }

    public static boolean inRange(double num, double low, double high) {
        return low <= num && num <= high;
    }

    public static String joinStrings(String delim, List<?> strings) {
        StringBuilder sb = new StringBuilder();
        for (int i = 0; i < strings.size(); ++i) {
            sb.append(strings.get(i).toString());
            if (i < strings.size() - 1) {
                sb.append(delim);
            }
        }
        return sb.toString();
    }

    public static boolean epsilonEquals(double a, double b, double epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static boolean epsilonEquals(double a, double b) {
        return epsilonEquals(a, b, kEpsilon);
    }

    public static boolean epsilonEquals(int a, int b, int epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static boolean allCloseTo(List<Double> list, double value, double epsilon) {
        boolean result = true;
        for (Double value_in : list) {
            result &= epsilonEquals(value_in, value, epsilon);
        }
        return result;
    }

    @Deprecated
    public static int convertRPMToNativeUnits(double rpm) {
        //return (int)(rpm * TechConstants.kSensorUnitsPerRotation / TechConstants.k100msPerMinute);
        return -1;
    }

    @Deprecated
    public static int convertNativeUnitsToRPM(double nativeUnits) {
        //return (int)(nativeUnits / TechConstants.kSensorUnitsPerRotation * TechConstants.k100msPerMinute);
        return -1;
    }

}