package states;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public abstract class AState
{
    protected long mEntryTime;
    protected Object mEntryParameter;

    protected void EnterState(Object pEntryParameter)
    {
        mEntryTime = System.currentTimeMillis();
        mEntryParameter = pEntryParameter;
    }

    protected void ExitState()
    {

    }

    protected abstract NextStateInfo Run();
    
    protected long GetTimeSinceEntry()
    {
        return System.currentTimeMillis() - mEntryTime;
    }

    protected boolean HasTimeElapsedSinceEntry(long pTime)
    {
        return GetTimeSinceEntry() > pTime;
    }

    protected abstract States GetState();
    protected abstract String GetName();

    protected void logStateInfo()
    {

    }
    
    protected void log()
    {
        SmartDashboard.putString("CurrentState", GetName());
        logStateInfo();
    }



}
