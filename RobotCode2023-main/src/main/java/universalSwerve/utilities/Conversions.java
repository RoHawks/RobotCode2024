package universalSwerve.utilities;

public class Conversions 
{
    public static double InchesToMeters(double pInches)
    {
        return pInches * 0.0254;
    }
    
    public static double InchesPerSecondToPetersPerSecond(double pInchesPerSecond)
    {
        return InchesToMeters(pInchesPerSecond);
    }


    public static double MetersPerSecondToInchesPerSecond(double pMetersPerSecond)
    {
        return MetersToInches(pMetersPerSecond);
    }

    public static double MetersToInches(double pMeters)
    {
        return pMeters / 0.0254;
    }

    public static double InchesPerSecondToRPMS(double pInchesPerSecond, double pDiameter)
    {
        double inchesPerRotation = Math.PI * pDiameter;
        double rotationsPerSecond = pInchesPerSecond  / inchesPerRotation;
        double rotationsPerMinute = rotationsPerSecond * 60.0;
        return rotationsPerMinute;
    }

    public static double RPMstoInchesPerSecond(double pRPMS, double pDiameter)
    {
        double inchesPerRotation = Math.PI * pDiameter;
        double rotationsPerSecond = pRPMS / 60.0;
        double inchesPerSecond = inchesPerRotation *  rotationsPerSecond;
        return inchesPerSecond;
    }

    public static double RPMsToFalconVelocityUnit(double pRPMS)
    {
        //Falcon velocity unit is "position change per 100ms" for some stupid reason
        //I think "position" here refers to encoder ticks
        //Of which there are 2048 per rotation
        double rotationsPer100ms = pRPMS / 60.0 / 10.0;
        double positionsPerRotation = 2048.0;
        double falconUnits = positionsPerRotation * rotationsPer100ms;
        return falconUnits;
    }

    public static double FalconVelocityUnitToRPMS(double pFalconVelocityUnit)
    {
        //Falcon velocity unit is "position change per 100ms" for some stupid reason
        //I think "position" here refers to encoder ticks
        //Of which there are 2048 per rotation
        double positionsPerRotation = 2048.0;
        double rotatitionsPer100ms = pFalconVelocityUnit / positionsPerRotation;
        double rpms = rotatitionsPer100ms * 60.0 * 10.0;
        return rpms;
    }
}
