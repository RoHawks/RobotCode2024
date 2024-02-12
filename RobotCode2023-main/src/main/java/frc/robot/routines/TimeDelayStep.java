package frc.robot.routines;

import java.util.List;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class TimeDelayStep extends AComponentStep {

    private long mTimeMs;
    private boolean mSkipThisStep;

    public TimeDelayStep(String pName, List<StepPrequisite> pPrerequisites, long pTimeMs, boolean pSkipThisStep)
    {
        super(pName, pPrerequisites);
        mTimeMs = pTimeMs;
        mSkipThisStep = pSkipThisStep;
    }

    @Override
    public void Run() {
        
    }

    @Override
    public double DistanceFromCompletion() {
        if(IsComplete() || mSkipThisStep)
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
        if(mSkipThisStep)
        {
            return true;
        }
        SmartDashboard.putNumber(mName + "_ElapsedTime", (System.currentTimeMillis() - mEntryTime));
        boolean returnValue =  (System.currentTimeMillis() - mEntryTime) > mTimeMs;
       
       /*  if(returnValue)
        {
            mHasBeenEnded = true;
        }
        */
        return returnValue;
        
    }
    
    @Override
    public String GetLogInfo() {
        return Long.toString(System.currentTimeMillis() - this.mEntryTime);
    }
}
