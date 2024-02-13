package universalSwerve.hardware.implementations;

import com.kauailabs.navx.frc.AHRS;

import universalSwerve.hardware.IGyroscope;

public class NavX implements IGyroscope 
{
    private AHRS mNavX;
    private boolean mIsFlipped;
    private double mOffset = 0;
    private Axis mAxis;

    public enum Axis
    {
        Yaw,
        Pitch,
        Roll
    }

    public NavX(boolean pIsFlipped, Axis pAxis)
    {
        mNavX = new AHRS();
        mAxis = pAxis;
        mIsFlipped = pIsFlipped;
    }

    public void SetCurrentAngle(double pAngle)
    {
        double mCurrentAngleAsReadByGetGyroInOurCoordinateSystem = GetGyroInOurCoordinateSystem();
        double mDifference = pAngle - mCurrentAngleAsReadByGetGyroInOurCoordinateSystem;
        mOffset += mDifference;
    }

    private double GetRawGyroAngle()
	{
        switch (mAxis)
        {
            case Yaw:
                return mNavX.getYaw();
            case Pitch:
                return mNavX.getPitch();
            case Roll:
                return mNavX.getRoll();
            default:
                throw new RuntimeException("Unknown Axis for NavX");
        }
	}

    

    private double GetGyroInOurCoordinateSystem()
	{
		return (((
        (mIsFlipped ? -1.0 : 1.0)
        *    
        GetRawGyroAngle()) % 360.0) + 360.0 + mOffset) % 360.0;
	}


    @Override
    public double GetCurrentAngle() {
        return GetGyroInOurCoordinateSystem();
    }
    
    
}
