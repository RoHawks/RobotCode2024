package robotcode.autonomous.steps;

import robosystems.Shooter;
import robotcode.autonomous.AAction;
import states.JoystickControlsWithSwerve;
import states.ShooterMode;

public class WaitAction extends AAction
{
    private long mTime;

    public WaitAction(long pTime)
    {
        mTime = pTime;
    }

    @Override
    public boolean Run() 
    {
        return HasTimeElapsed(mTime);

        
    }

    @Override
    public void EndAction() {
        
    }
    
    
}
