package states;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.routines.Routine;
import frc.robot.routines.ScoringRoutines;
import robosystems.Arm;
import robosystems.Intake;
import robosystems.Intake.IntakeMode;
import universalSwerve.SwerveDrive;

public class ScoringState extends AState{

    private SwerveDrive mSwerveDrive;
    private Intake mIntake;
    
    private Controls mControls;
    private Routine mScoringRoutine;

    public ScoringState(
        SwerveDrive pSwerveDrive,
        Intake pIntake,        
        Controls pControls)
    {
        mIntake = pIntake;
        mSwerveDrive = pSwerveDrive;        
        mControls = pControls;   
    }

    
    @Override
    protected NextStateInfo Run() {

        mIntake.SetMode(IntakeMode.Holding);
        mIntake.Run(mControls.GetIsLowGoalMode());
        mIntake.deploy();
        mSwerveDrive.Run(mControls);
        SmartDashboard.putNumber("calling score swerve at ", System.currentTimeMillis());

        mScoringRoutine.Run();
        

        //I think if we press the "Intake" button we need to abort the squence and go back to intaking, I guess.
        //Need to implement that
        
        if(mScoringRoutine.IsComplete())
        {
            return new NextStateInfo(States.Intaking);
        }
        else
        {
            return new NextStateInfo(States.Scoring); //Keep on scorin'
        }
    }

    @Override
    protected States GetState() {
        return States.Scoring;
    }

    @Override
    protected String GetName() {
        
        return "Scoring";
    }


    private double GetAngleFromLimelight()
    {
        double limelightAngle = 0;
        try
        {
            limelightAngle = NetworkTableInstance.getDefault().getTable("limelight").getValue("llrobot").getDoubleArray()[0];
        }
        catch(Exception e)
        {
           //System.out.println("Couldn't get limelight Angle");
        }

        return limelightAngle;
        
    }

    @Override
    protected void EnterState(Object pEntryParameter)
    {
        super.EnterState(pEntryParameter);
        ScoringInstructions scoringInstructions = (ScoringInstructions)pEntryParameter;
        mScoringRoutine = ScoringRoutines.GetInstance().GetRoutineForInstructions(scoringInstructions);
        mScoringRoutine.Reset(
            scoringInstructions.GetManualAimOverride() ? scoringInstructions.GetManualAimDirection() : GetAngleFromLimelight()

        );
    }
    
}
