package universalSwerve.components;
import java.util.Hashtable;

import edu.wpi.first.math.geometry.Translation2d;
import universalSwerve.utilities.AngleUtilities;
import universalSwerve.utilities.Conversions;


public class Wheel 
{
    private WheelLabel mLabel;
    private double mXOffsetFromCenter; //inches.  Negatives are left of center of rotation, positives are right of center of rotation
    private double mYOffsetFromCenter; //inches.  Negatives are behind the center of rotation, positives are in front of center of rotation
    private ITranslationSystem mTranslationSystem;
    private IRotationSystem mRotationSystem;
    private double mTargetAngle;

    public Wheel(WheelLabel pLabel, double pXOffsetFromCenter, double pYOffsetFromCenter,
    ITranslationSystem pTranslationSystem, IRotationSystem pRotationSystem)
    {
        mLabel = pLabel;
        mXOffsetFromCenter = pXOffsetFromCenter;
        mYOffsetFromCenter = pYOffsetFromCenter;
        mTranslationSystem = pTranslationSystem;
        mRotationSystem = pRotationSystem;
        mTargetAngle = 0;

    }

    public WheelLabel GetWheelLabel()
    {
        return mLabel;
    }

    public double GetXOffsetFromCenter()
    {
        return mXOffsetFromCenter;
    }

    public double GetYOffsetFromCenter()
    {
        return mYOffsetFromCenter;
    }

    public double GetCurrentAngle()
    {
        return mRotationSystem.GetCurrentAngle();
    }

    public double GetRawCurrentAngle()
    {
        return mRotationSystem.GetRawCurrentAngle();
    }

    public void SetToBreakMode()
    {
        mTranslationSystem.SetToBreakMode();
    }

    public void SetToCoastMode()
    {
        mTranslationSystem.SetToCoastMode();
    }


    /*
    Sets the angle that we want the wheel to truly be pointing at, relative to the robot chassis.  If the wheel should be revesed it is already baked into this numer
    */
    public void SetWheelTargetAngle(double pAngle)
    {
        mTargetAngle = pAngle;
        mRotationSystem.SetAngle(pAngle);
    }

    
    /*
    Sets the speed that we want the wheel to be truly running at.  If the wheel should be revesed it is already baked into this number
    Positive means "forward if the wheel is pointed in the designated forwards direction"
    */
    public void SetWheelVelocity(double pSpeed)
    {
        mTranslationSystem.SetVelocity(pSpeed);
    }

    public void StopEverything()
    {
        mRotationSystem.StopEverything();
        mTranslationSystem.StopEverything();
    }

    public void StopTranslationButAllowWheelDirection()
    {
        mTranslationSystem.StopEverything();
    }

    public Translation2d GetCenterToWheelTranslation()
    {
        return new Translation2d(Conversions.InchesToMeters(this.mYOffsetFromCenter),  -Conversions.InchesToMeters(this.mXOffsetFromCenter)); //This transaltion 2D system seems to think that X is "forward" and Y is "right", so switch it here to bring it inline with how we think
    }

    public void Initialize()
    {
        mRotationSystem.Initialize();
        mTranslationSystem.Initialize();
    }

    public boolean IsCloseToTargetAngle()
    {
        return AngleUtilities.AbsoluteDistanceBetweenAngles(mTargetAngle, mRotationSystem.GetCurrentAngle()) < 10;
         
    }

    public double GetDistanceTravelled()
    {
        return mTranslationSystem.GetDistanceTravelled();
    }

    public void ResetDistanceTravelled()
    {
        mTranslationSystem.ResetDistanceTravelled();
    }

    public double GetPercentOutput()
    {
        return mTranslationSystem.GetPercentOutput();
    }

    public double GetVelocity()
    {
        return mTranslationSystem.GetVelocity();
    }
}
