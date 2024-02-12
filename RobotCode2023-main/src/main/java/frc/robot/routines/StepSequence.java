package frc.robot.routines;


import java.util.List;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class StepSequence {
    private List<AComponentStep> mComponentSteps;
    private int mCurrentStepNumber;
    private String mName;
    private String mLastLogStatement = "";

    public StepSequence(String pName, List<AComponentStep> pComponentSteps)
    {
        mComponentSteps = pComponentSteps;
        mCurrentStepNumber = 0;
        mName = pName;

    }

    public String GetName()
    {
        return mName;
    }

    public String GetStatusForLog()
    {
        return mLastLogStatement; // + ";" + mComponentSteps.get(mCurrentStepNumber).GetLogInfo();
    }

    public void Run()
    {
        AComponentStep currentStep = mComponentSteps.get(mCurrentStepNumber);
        /*
        SmartDashboard.putString(mName + "_CurrentStep", currentStep.Name());
        SmartDashboard.putBoolean(mName + "_CurrentStepIsComplete", currentStep.IsComplete());
        SmartDashboard.putBoolean(mName + "_CurrentStepAllPrerequisitesComplete", currentStep.AllPrerequisitesComplete());
        SmartDashboard.putBoolean(mName + "_CurrentStepHasBeenStarted", currentStep.HasBeenStarted());
        */
        
        
        //this should always be the case, except for initial steps.  So check here for initial steps (and all steps, might as well)
        if(currentStep.AllPrerequisitesComplete())
        {
            if(currentStep.IsComplete())
            {
                boolean thisStepPrereqsComplete =currentStep.AllPrerequisitesComplete();
                boolean nextStepPrereqsComplete = mCurrentStepNumber < mComponentSteps.size() - 1
                    && mComponentSteps.get(mCurrentStepNumber + 1).AllPrerequisitesComplete();
                mLastLogStatement = "Completed " + currentStep.Name() + "; thisStepPrereqsComplete = " + thisStepPrereqsComplete + "; NextStepPreqsComplete = " + nextStepPrereqsComplete;
                //mLastLogStatement = "Completed " + currentStep.Name();// + "; thisStepPrereqsComplete = " + thisStepPrereqsComplete + "; NextStepPreqsComplete = " + nextStepPrereqsComplete;
                //SmartDashboard.putString(mName + "_Status", "Completed" + currentStep.Name() + ", thisStepPrereqsComplete = " + thisStepPrereqsComplete + ", NextStepPreqsComplete = " + nextStepPrereqsComplete);    
            }
            else
            {
                mLastLogStatement = "Running " + currentStep.Name();
            }

            SmartDashboard.putString(mName + "_Status",  mLastLogStatement);

            if(mCurrentStepNumber == 0 && !currentStep.HasBeenStarted())
            {
                currentStep.EnterStep();
            }
            currentStep.Run();
            
        }
        else
        {
            mLastLogStatement = "Waiting for prereqs for " + currentStep.Name();
            SmartDashboard.putString(mName + "_Status",  mLastLogStatement);
        }
        if(currentStep.IsComplete()
            && mCurrentStepNumber < mComponentSteps.size() - 1 //there is another step
            && currentStep.AllPrerequisitesComplete()
            && mComponentSteps.get(mCurrentStepNumber + 1).AllPrerequisitesComplete()
            )
        {
            currentStep.EndStep();
            mCurrentStepNumber++;
            AComponentStep newStep = mComponentSteps.get(mCurrentStepNumber);
            newStep.EnterStep();
        }
        if(currentStep.IsComplete() && ! currentStep.AllPrerequisitesComplete())
        {
            //System.out.println(mName + " is waiting on a prereq before moving on.");
        }
    }

    public boolean IsCompete()
    {
        return mCurrentStepNumber == mComponentSteps.size() - 1
        && mComponentSteps.get(mCurrentStepNumber).IsComplete();
    }
    
    public void Reset(double pClawAngle)
    {
        for(int i = 0; i < mComponentSteps.size(); i++)
        {
            mComponentSteps.get(i).Reset(pClawAngle);
        }

        mCurrentStepNumber = 0;
    }
}
