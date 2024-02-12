package robosystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.jni.CANSparkMaxJNI;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class Arm {

    private CANSparkMax mUpperJointSparkMax;
    private CANSparkMax mLowerJointSparkMax;
    private DoubleSolenoid mClampSolenoid;

    public Arm(CANSparkMax pUpperJointSparkMax, CANSparkMax pLowerJointSparkMax, DoubleSolenoid pClampSolenoid)
    {
        mUpperJointSparkMax = pUpperJointSparkMax;
        mLowerJointSparkMax = pLowerJointSparkMax;
        mClampSolenoid = pClampSolenoid;
    }

    public void NormalHoldingStill()
    {
        mClampSolenoid.set(Value.kForward);
        mLowerJointSparkMax.set(0);//maybe in the future, PID to hold?  But beware of the clamp being close
        mUpperJointSparkMax.set(0);//maybe in the future, PID to hold?  But beware of the clamp being close
    }
    
    public void CloseClamp()
    {
        mClampSolenoid.set(Value.kForward);
    }

    public void OpenClamp()
    {
        mClampSolenoid.set(Value.kReverse);
    }
}
