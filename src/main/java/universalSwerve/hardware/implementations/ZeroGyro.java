package universalSwerve.hardware.implementations;

import universalSwerve.hardware.IGyroscope;

/*
 * This class is meant to be a stub for testing.
 * Alternately, if your gyro is broken, you can use this and you'll just 
 * operate in a non-field relative way
 */
public class ZeroGyro implements IGyroscope{

    @Override
    public double GetCurrentAngle() {
        
        return 0;
    }

    @Override
    public void SetCurrentAngle(double pAngle) {
        
    }
    
}
