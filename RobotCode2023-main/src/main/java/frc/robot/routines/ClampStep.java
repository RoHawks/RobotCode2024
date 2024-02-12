package frc.robot.routines;

import java.util.List;

import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;

public class ClampStep extends AComponentStep{


    private DoubleSolenoid mSolenoid;
    private boolean mShouldClose;
    private long TIME_TO_COMPLETE_MS = 50;

    public ClampStep(String pName, List<StepPrequisite> pPrerequisites, DoubleSolenoid pSolenoid, boolean pShouldClose)
    {
        super(pName, pPrerequisites);
        mSolenoid = pSolenoid;
       
        mShouldClose = pShouldClose;
    }

    @Override
    public void Run() {
        
        mSolenoid.set(mShouldClose ? Value.kForward : Value.kReverse);
        
        
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
        if(mHasBeenCompleted)
        {
            return true;
        }        
        mHasBeenCompleted = ((System.currentTimeMillis() - this.mEntryTime) > TIME_TO_COMPLETE_MS);
        return mHasBeenCompleted;
    }

    @Override
    public String GetLogInfo() {
        return Long.toString(System.currentTimeMillis() - this.mEntryTime);
    }
    
}
