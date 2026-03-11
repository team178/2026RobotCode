package frc.robot.util;

import java.util.Map;
import java.util.TreeMap;

public class HubShootLUT {
    private static final TreeMap<Double, Double> flywheelSpeedTable = new TreeMap<>();

    static {

    }

    public static double getFlywheelSpeedAtDistance(double targetDistance) {
        if (flywheelSpeedTable.isEmpty()) {
            return 0.0;
        }

        Map.Entry<Double, Double> lowerPoint = flywheelSpeedTable.floorEntry(targetDistance);
        Map.Entry<Double, Double> upperPoint = flywheelSpeedTable.ceilingEntry(targetDistance);

        // if the point is out of bounds, return nearest point
        if (lowerPoint == null) return upperPoint.getValue();
        if (upperPoint == null) return lowerPoint.getValue();

        // if the target distance was directly in the data, and hence the two nearest points are the same, return one of the two
        if (lowerPoint.getKey().equals(upperPoint.getKey())) return lowerPoint.getValue();

        // standard interpolation
        double x1 = lowerPoint.getKey();
        double y1 = lowerPoint.getValue();
        double x2 = upperPoint.getKey();
        double y2 = upperPoint.getValue();

        return y1 + (targetDistance - x1) * ((y2 - y1) / (x2 - x1));
    }
}
