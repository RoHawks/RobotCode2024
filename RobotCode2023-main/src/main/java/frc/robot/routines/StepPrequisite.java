package frc.robot.routines;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class StepPrequisite {
    private AComponentStep mPrequisite;
    private double mPassPositiveBound;
    private double mPassNegativeBound;

    public StepPrequisite(AComponentStep pPrerequisite, double pPassPositiveBound, double pPassNegativeBound)
    {
        mPrequisite = pPrerequisite;
        mPassPositiveBound = pPassPositiveBound;
        mPassNegativeBound = pPassNegativeBound;
    }

    public boolean IsCompleteEnough()
    {
        if(mPrequisite.Name().equals("PauseBeforeLowerUpToScore"))
        {
            SmartDashboard.putNumber("StepPrereq_DistanceToComplete", mPrequisite.DistanceFromCompletion());
            SmartDashboard.putBoolean("StepPrereq_HasBeenStarted", mPrequisite.HasBeenStarted());
        }
        
        double distanceFromComplete = mPrequisite.DistanceFromCompletion();
        //System.out.println("distanceFromCompete " + distanceFromComplete + ", hasBeenEnded" + mPrequisite.HasBeenEnded());
        return mPrequisite.HasBeenStarted()
        && 
        (mPrequisite.HasBeenEnded() || 
        (distanceFromComplete <= mPassPositiveBound && distanceFromComplete >= mPassNegativeBound));
    }
    
    
}
