package robosystems;

import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Lights 
{
    private AddressableLED mLeds;
    private AddressableLEDBuffer mLedBuffer;
    private int mCurrentRed;
    private int mCurrentGreen;
    private int mCurrentBlue;
    private boolean mHasStarted = false;

    public Lights()
    {
        mLeds = new AddressableLED(9);
        mLedBuffer = new AddressableLEDBuffer(18);
        mLeds.setLength(mLedBuffer.getLength());    

    }
  
    public void Run()
    {

        SmartDashboard.putString("LED", "" + mCurrentRed + "," + mCurrentGreen + "," + mCurrentBlue);
        for (var i = 0; i < mLedBuffer.getLength(); i++) 
        {      
            mLedBuffer.setRGB(i, mCurrentRed, mCurrentGreen, mCurrentBlue);
        }
        mLeds.setData(mLedBuffer);    
        mLeds.start();    
        
    }

    public enum LightingScheme
    {
        Off,
        Intaking,
        HoldingButNoCameraLock,
        HoldingWithCameraLock,
        Shooting
    }

    public void SetLightingScheme(LightingScheme pLightingScheme)
    {
        switch(pLightingScheme)
        {
            case Off:
                mCurrentRed = 0;
                mCurrentGreen = 0;
                mCurrentBlue = 0;
                break;
            case Intaking:
                mCurrentRed = 0;
                mCurrentGreen = 0;
                mCurrentBlue = 255;
                break;
            case HoldingButNoCameraLock:
                mCurrentRed = 255;
                mCurrentGreen =242;
                mCurrentBlue = 0;
                break;
            case HoldingWithCameraLock:
                mCurrentRed = 0;
                mCurrentGreen = 255;
                mCurrentBlue = 0;
                break;
            case Shooting:
                mCurrentRed = 255;
                mCurrentGreen = 0;
                mCurrentBlue = 255;
                break;

        }
    }

    
}
