package states;

import robosystems.Intake;
import robosystems.Intake.IntakeMode;
import universalSwerve.SwerveDrive;

public class LowGoalScoringState extends AState{


    private SwerveDrive mSwerveDrive;
    private Intake mIntake;
    private Controls mControls;

    public LowGoalScoringState(SwerveDrive pSwerveDrive, Intake pIntake, Controls pControls)
    {
        mSwerveDrive = pSwerveDrive;
        mIntake = pIntake;
        mControls = pControls;
    }

    @Override
    protected NextStateInfo Run() 
    {
        mIntake.SetMode(IntakeMode.GracefulEjecting);
        mIntake.Run(mControls.GetIsLowGoalMode());
        mIntake.deploy();
        mSwerveDrive.Run(mControls);

        if(mControls.GetForceIntakingMode())
        {
            return new NextStateInfo(States.Intaking);
        }
        else
        {
            return new NextStateInfo(States.LowGoalScoring);//Keep on LowGoalScorin'
        }
    }

    @Override
    protected States GetState() {
        return States.LowGoalScoring;
    }

    @Override
    protected String GetName() {
        return "LowGoalScoring";
    }
    
    
}
