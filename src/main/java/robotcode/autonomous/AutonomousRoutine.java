package robotcode.autonomous;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class AutonomousRoutine {

    private ArrayList<Step> mSteps;
    private int mCurrentStep =0;
    private boolean mIsFirstTime = true;
    private String mName;

    public AutonomousRoutine(String pName)
    {
        mSteps = new ArrayList<Step>();
        mName = pName;
    }

    public String GetName()
    {
        return mName;
    }

    public void AddStep(Step pStep)
    {
        mSteps.add(pStep);
    }

    public void Reset()
    {
        mCurrentStep = 0;
        
    }


    public void Run()
    {
        Step currentStep = mSteps.get(mCurrentStep);
        SmartDashboard.putString("AutonomousStep", currentStep.GetName());
        
        if(mIsFirstTime)
        {        
            currentStep.EnterStep();
            mIsFirstTime =false;
        }
        boolean hasStepCompleted = currentStep.Run();
        if(hasStepCompleted)
        {
            if(mCurrentStep < mSteps.size() - 1) //we never really should go past the finish step, but just in case we forgot to put a finish step, just don't advance to the next one
            {

                currentStep.EndStep();
                mCurrentStep++;

                Step newStep = mSteps.get(mCurrentStep);
                newStep.EnterStep();
            }
        }
    }
    
}
