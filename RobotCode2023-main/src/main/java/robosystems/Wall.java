package robosystems;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Wall 
{
    private DoubleSolenoid mSolenoid;

    public Wall(DoubleSolenoid pSolenoid)
    {
        mSolenoid = pSolenoid;
    }

    public void SetIfLowGoalMode(boolean pIsLowGoalMode)
    {
        if(pIsLowGoalMode)
        {
            Extend();
        }
        else
        {
            Retract();
        }
    }

    public void Extend()
    {
        mSolenoid.set(Value.kReverse);
    }

    public void Retract()
    {
        mSolenoid.set(Value.kForward);
    }
    
}
