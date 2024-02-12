package frc.robot.routines;

import java.util.List;

import edu.wpi.first.util.datalog.StringLogEntry;
import edu.wpi.first.wpilibj.DataLogManager;

public class Routine 
{
    private List<StepSequence> mStepSequences;
    private long mStartTime = 0;
    private StringLogEntry mRoutineLog;

    private boolean DO_LOGGING = false;

    public Routine(List<StepSequence> pStepSequences)
    {
        mStepSequences = pStepSequences;
        if(DO_LOGGING)
        {
            mRoutineLog = new StringLogEntry(DataLogManager.getLog(), "/my/routine");
        }
        else
        {
            mRoutineLog = null;
        }
    }

    private String mLastLog = "";
    public void Run()
    {
        if(mStartTime == 0)
        {
            //first time since reset
            mStartTime = System.currentTimeMillis();
        }
        
        StringBuilder log = new StringBuilder();

        for(int i = 0; i < mStepSequences.size(); i++)
        {
            StepSequence stepSequence = mStepSequences.get(i);
            stepSequence.Run();

            if(DO_LOGGING)
            {
                log.append("[");
                log.append(stepSequence.GetName());
                log.append(":");
                log.append(stepSequence.GetStatusForLog());
                log.append("];");
            }
            
            
        }
        
        if(DO_LOGGING)
        {
            String stepLog = log.toString();
            if(!mLastLog.equals(stepLog))
            {
                
                String logMessage = Long.toString((System.currentTimeMillis() - mStartTime)) + "," + stepLog;
                mRoutineLog.append(logMessage);
                
                mLastLog = stepLog;
            }
        }
        
    } 
    
    public void Reset(double pClawAngle)
    {
        mStartTime = 0;
        mLastLog = "";
        for(int i = 0; i < mStepSequences.size(); i++)
        {
            StepSequence stepSequence = mStepSequences.get(i);
            stepSequence.Reset(pClawAngle);
        }
    }

    public boolean IsComplete()
    {
        for(int i = 0; i < mStepSequences.size(); i++)
        {
            StepSequence stepSequence = mStepSequences.get(i);
            if(!stepSequence.IsCompete())
            {
                return false;
            }
        }
        return true;
    }
    
}
