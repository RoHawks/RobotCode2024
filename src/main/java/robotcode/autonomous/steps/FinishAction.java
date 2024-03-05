package robotcode.autonomous.steps;

import robotcode.autonomous.AAction;
import universalSwerve.SwerveDrive;

public class FinishAction extends AAction{

    private SwerveDrive mSwerveDrive;

    public FinishAction(SwerveDrive pSwerveDrive)
    {
        mSwerveDrive = pSwerveDrive;
    }

    @Override
    public boolean Run() {
        mSwerveDrive.StopEverything();
        return false;
    }

    @Override
    public void EndAction() {
      
    }
    
}
