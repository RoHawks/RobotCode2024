package universalSwerve.utilities;

public class AngleUtilities 
{


	public static double AbsoluteDistanceBetweenAngles(double pAngleA, double pAngleB)
	{
		double angleANormalized = Normalize(pAngleA);
		double angleBNormalized = Normalize(pAngleB);
		double greaterAngle = Math.max(angleANormalized, angleBNormalized);
		double lesserAngle = Math.min(angleANormalized, angleBNormalized);
		double distance1 = greaterAngle - lesserAngle;
		double distance2 = 360 - greaterAngle + lesserAngle;
		return Math.min(distance1, distance2);
	}

    /*
        Takes and angle and brings it into the range [0, 360)
    */
    public static double Normalize(double pAngle)
    {
        if(pAngle >= 0.0)
        {
            return pAngle % 360.0;
        }
        else
        {
            return 360.0 + (pAngle % 360); //the mod on the right here will return a negative number, i.e. -11 % 5 == -1
        }
    }

    /*
        Takes an angle and gives the opposite angle, normalized.  So, for example, translates 90 degrees (right) to 270 degrees (left)
    */
    public static double Reverse(double pAngle)
    {
        return Normalize(pAngle + 180.0);      
    }
    
    public static double ConvertSwerveKinematicsAnglesToOurAngles(double pSwerveKinematicsAngle)
	{
		//For SwerveKinematics, 0 means straight ahead
		//-90 means right
		//90 means left
		//and -180, and also sometimes 180, mean backwards

		return ((-1.0 * pSwerveKinematicsAngle) + 360.0) % 360.0;

	}

    public static double ConvertOurAnglesToSwerveKinematicsAngles(double pOurAngle)
	{
		//inverse of ConvertSwerveKinematicsAnglesToOurAngles
		if(pOurAngle < 180)
		{
			return -1.0 * pOurAngle;
		}
		else
		{
			return 360.0 - pOurAngle;
		}
	}

    /*
            pXComponent [-1, 1]: 1 means "right"
            pYComponent [-1, 1]: 1 means "Forward"  
            return an angle in our coordinate system according to the vector passed in here.        
    */
    public static double ConvertLinearComponentsToAngle(double pXComponent, double pYComponent)
    {
		double val;

		double absX = Math.abs(pXComponent);
		double absY = Math.abs(pYComponent);

		if(absX == 0.0 && absY == 0.0)
		{
			return 0;
		}

		// double val = (Math.toDegrees(Math.atan(x/-y)) + 360) % 360;
		double tanValue = Math.toDegrees(Math.atan(absY / absX));

		if(pXComponent >= 0 && pYComponent <= 0) //QUAD NE
		{
			val = 90.0 - tanValue;
		}
		else if(pXComponent >= 0 && pYComponent > 0 )
		{
			val = 90 + tanValue;
		}
		else if(pXComponent <= 0 && pYComponent > 0)
		{
			val = 270 -  tanValue;
		}
		else
		{
			val = 270 + tanValue;
		}
		return val % 360.0;
	}
}
