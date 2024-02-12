package frc.robot.routines;

import java.util.List;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class ClawStep extends AComponentStep{


    //private DoubleSolenoid mLeftFinger;
    private DoubleSolenoid mRightFinger;
    private boolean mShouldClose;
    private long mTimeToComplete;

    public ClawStep(String pName, List<StepPrequisite> pPrerequisites, 
    //DoubleSolenoid pLeftFinger,
     DoubleSolenoid pRightFinger, boolean pShouldClose, long pTimeToComplete)
    {
        super(pName, pPrerequisites);
        //mLeftFinger = pLeftFinger;
        mRightFinger = pRightFinger;
        mShouldClose = pShouldClose;
        mTimeToComplete = pTimeToComplete;
    }

    @Override
    public void Run() {
        //System.out.println("Running Claw Step:  " + mName);
        //mLeftFinger.set(mShouldClose ? Value.kReverse : Value.kForward);
        mRightFinger.set(mShouldClose ? Value.kReverse : Value.kForward);
        
    }

    @Override
    public double DistanceFromCompletion() {        
        if(IsComplete())
        {
            return 0;
        }
        else
        {
            return 1000;
        }
    }

    @Override
    public boolean IsComplete() {
        
        if(!mHasBeenStarted)
        {
            return false;
        }
        /*
        if(mName.equals("Grab"))
        {
            SmartDashboard.putNumber("GrabTimerCheck", (System.currentTimeMillis() - this.mEntryTime));
        }
        */
        if(mHasBeenCompleted)
        {
            return true;
        }        
        mHasBeenCompleted = ((System.currentTimeMillis() - this.mEntryTime) > mTimeToComplete);
        return mHasBeenCompleted;
    }

    @Override
    public String GetLogInfo() {
        return Long.toString(System.currentTimeMillis() - this.mEntryTime);
    }
    
}
