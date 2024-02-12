package universalSwerve.components.implementations;

import com.revrobotics.CANSparkMax;

import universalSwerve.components.IRotationSystem;
import universalSwerve.hardware.implementations.LampreyWheelAngleReader;
import universalSwerve.utilities.AngleUtilities;
import universalSwerve.utilities.PIDFConfiguration;

public class SparkMaxAndLampreyRotationSystem extends SparkMaxAndAbsoluteEncoderRotationSystem
{

    private LampreyWheelAngleReader mLamprey;

    

    public SparkMaxAndLampreyRotationSystem(CANSparkMax pSparkMax, double pGearRatioBetweenMotorAndWheel, PIDFConfiguration pPidfConfiguration, LampreyWheelAngleReader pLamprey)
    {
        super(pSparkMax, pGearRatioBetweenMotorAndWheel, pPidfConfiguration);

        mLamprey = pLamprey;

    
    
    }

    public double GetRawCurrentAngle()
    {
        return GetCurrentAngleFromAbsoluteEncoder();
    }
    @Override
    protected double GetCurrentAngleFromAbsoluteEncoder() {
        return mLamprey.GetCurrentAngle();
    }




}
