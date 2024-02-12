package robosystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.RelativeEncoder;

public class Wrist
{
    private CANSparkMax mWristMotor;

    public Wrist(CANSparkMax pWristMotor)
    {
        mWristMotor = pWristMotor;
        
    }

    /** Calls CANSparkMax set method */
    public void wristSet(double pAmount)
    {
        throw new RuntimeException("Not implemented yet... actually called via the Routine");
    }

    
}