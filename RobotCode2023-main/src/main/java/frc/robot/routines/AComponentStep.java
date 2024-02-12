package frc.robot.routines;

import java.util.ArrayList;
import java.util.List;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public abstract class AComponentStep 
{
    protected long mEntryTime;
    private List<StepPrequisite> mPrerequisites;
    protected String mName;
    protected boolean mHasBeenEnded = false;
    protected boolean mHasBeenStarted = false;
    protected boolean mHasBeenCompleted =false;
    

    public AComponentStep(String pName, List<StepPrequisite> pPrerequisites)
    {
        mName = pName;
        mPrerequisites = pPrerequisites;
        if(mPrerequisites == null)//just allow caller to pass in null to mean no prereqs
        {
            mPrerequisites = new ArrayList<StepPrequisite>();
        }
    }

    public void AddPrerequisite(StepPrequisite pStepPrequisite)
    {
        mPrerequisites.add(pStepPrequisite);
    }

    public String Name()
    {
        return mName;
    }

    public boolean AllPrerequisitesComplete()
    {
        if(mName.equals("lowerJointScoochBackwardsToAllowArmUp2"))
        {
            SmartDashboard.putNumber("AllPrerequisitesComplete_NumPrereqs", mPrerequisites.size());
            
            //System.out.println("Number of prereqs:" + mPrerequisites.size());
        }
        for(int i = 0; i < mPrerequisites.size(); i++)
        {
            if(!mPrerequisites.get(i).IsCompleteEnough() )
            {
                return false;
            }
        }
        return true;
    }

    public void EnterStep()
    {
        mEntryTime = System.currentTimeMillis();
        mHasBeenStarted = true;
    }

    public abstract void Run();

    public abstract double DistanceFromCompletion();

    public void EndStep()
    {
        mHasBeenEnded = true;
    }

    public long GetTimeSinceEntry()
    {
        return System.currentTimeMillis() - mEntryTime;
    }

    public abstract boolean IsComplete();

    public void Reset(double pClawAngle)
    {
        mHasBeenEnded = false;
        mHasBeenStarted = false;
        mHasBeenCompleted = false;
    }

    public boolean HasBeenEnded()
    {
        return mHasBeenEnded;
    }

    public boolean HasBeenStarted()
    {
        return mHasBeenStarted;
    }
    
    public abstract String GetLogInfo();
}
