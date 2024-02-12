package universalSwerve.hardware.implementations;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import universalSwerve.hardware.IGyroscope;

public class ADIS16470Gyro implements IGyroscope{


    private ADIS16470_IMU mADIS16470;
    private boolean mIsFlipped;
    private double mOffset = 0;

    public ADIS16470Gyro(IMUAxis pAxis, boolean pIsFlipped)
    {
        mADIS16470 = new ADIS16470_IMU();
        // mADIS16470.setYawAxis(pAxis);
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
        return 0;
		// return mADIS16470.getYawAxis().GetRawGyroAngle();
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
