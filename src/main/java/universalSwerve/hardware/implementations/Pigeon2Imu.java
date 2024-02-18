package universalSwerve.hardware.implementations;

import com.ctre.phoenix6.hardware.core.CorePigeon2;

import edu.wpi.first.wpilibj.ADIS16470_IMU;
import edu.wpi.first.wpilibj.ADIS16470_IMU.IMUAxis;
import universalSwerve.hardware.IGyroscope;

public class Pigeon2Imu implements IGyroscope{


    private CorePigeon2 mCorePigeon2;
    private boolean mIsFlipped;
    private double mOffset = 0;
    private IMUAxis mYawAxis; 

    public Pigeon2Imu(boolean pIsFlipped)
    {
        mCorePigeon2 = new CorePigeon2(22);
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
		return mCorePigeon2.getYaw().getValueAsDouble();
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
