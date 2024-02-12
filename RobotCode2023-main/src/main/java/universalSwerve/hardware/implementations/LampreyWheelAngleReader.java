package universalSwerve.hardware.implementations;

import edu.wpi.first.wpilibj.Counter;
import universalSwerve.hardware.*;
import universalSwerve.utilities.AngleUtilities;

public class LampreyWheelAngleReader implements IWheelAngleReader
{

    private Counter mLampreyDevice;
    private double mOffset; //this is the number that the device returns when pointing straight ahead

    public LampreyWheelAngleReader(int pChannel, double pOffset)
    {
        mLampreyDevice = new Counter(pChannel);
		mLampreyDevice.setSemiPeriodMode(true);
		mLampreyDevice.setSamplesToAverage(1);
        mOffset = pOffset;
    }

    private double GetLampreyRawValue()
	{
		return mLampreyDevice.getPeriod();
	}

	private double GetLampreyAdjustedAngle()
	{
		double MAX_LAMPREY_READING = 0.001363;
        double percentOfRotation = GetLampreyRawValue() / MAX_LAMPREY_READING;
        return AngleUtilities.Normalize(percentOfRotation * 360.0);
        
	}

    @Override
    public double GetCurrentAngle() {
        return GetLampreyAdjustedAngle();
    }

    
    private double mInitialAngle; 

    public void Initialize() {
        mInitialAngle = GetCurrentAngle();
        
    }

    
}