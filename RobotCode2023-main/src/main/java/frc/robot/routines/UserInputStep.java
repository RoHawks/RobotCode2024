package frc.robot.routines;

import java.util.List;

import states.Controls;

public class UserInputStep extends AComponentStep{

    private boolean mHasButtonBeenPressed = false;
    private Controls mControls;

    public UserInputStep(String pName, List<StepPrequisite> pPrerequisites, Controls pControls)
    {
        super(pName, pPrerequisites);
        mControls = pControls;
    }

    @Override
    public void Run() {
        
        if(mControls.GetReleaseGamePiece())
        {
            mHasButtonBeenPressed = true;
        }
    }

    @Override
    public double DistanceFromCompletion() {
        
        return IsComplete() ? 0 : 1000;
    }

    @Override
    public boolean IsComplete() {
        
        return mHasButtonBeenPressed;
    }

    public void Reset(double pClawAngle)
    {
        super.Reset(pClawAngle);
        this.mHasButtonBeenPressed =false;
    }

    @Override
    public String GetLogInfo() {
        return IsComplete() ? "Complete" : "Waiting";
    }
    
}
