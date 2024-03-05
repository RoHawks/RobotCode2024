package robotcode.autonomous.steps;

import robotcode.autonomous.Step;
import universalSwerve.SwerveDrive;

public class FinishStep extends Step{

    private SwerveDrive mSwerveDrive;
    public FinishStep(SwerveDrive pSwerveDrive) {
        super("Finish");
        mSwerveDrive = pSwerveDrive;
        AddAction(new FinishAction(mSwerveDrive));
        
    }


    
}
