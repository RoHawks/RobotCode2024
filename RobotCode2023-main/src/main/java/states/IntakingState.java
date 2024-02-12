package states;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import robosystems.Arm;
import robosystems.Intake;
import robosystems.Wall;
import robosystems.Intake.IntakeMode;
import universalSwerve.SwerveDrive;

public class IntakingState extends AState {

    private SwerveDrive mSwerveDrive;
    private Intake mIntake;
    private Arm mArm;
    private Controls mControls;
    private boolean mLastWasInvokingScoringRoutine = false;
    private long mStartedInvokingScoringRoutineTime = 0;
    private static long SCORING_BUTTON_MIN_PRESS_TIME = 100;//ms
    private boolean mManualAimOverride = false;
    private double mManualAimDirection = 0;
    private Wall mWall;

    private GamePieceLocation mGamePieceLocation = GamePieceLocation.MID;

    public IntakingState(
        SwerveDrive pSwerveDrive,
        Intake pIntake,
        Arm pArm,
        Controls pControls,
        Wall pWall)
        {
            mSwerveDrive = pSwerveDrive;
            mIntake = pIntake;
            mArm = pArm;
            mControls = pControls;
            mWall = pWall;
            
        }
        
    @Override
    protected NextStateInfo Run() {
        
        mIntake.Run(mControls.GetIsLowGoalMode());
        mIntake.deploy();
        

        mSwerveDrive.Run(mControls);
        mArm.NormalHoldingStill();
        mWall.SetIfLowGoalMode(mControls.GetIsLowGoalMode());

        if(mControls.GetForceHoldingMode())
        {
            mIntake.SetMode(IntakeMode.Holding);
        }
        else if(mControls.GetForceIntakingMode())
        {
            mIntake.SetMode(IntakeMode.NormalIntaking);
        }
        else if(mControls.GetForceEjectionMode())
        {
            mIntake.SetMode(IntakeMode.Ejecting);
        }
        

        if(mControls.GetPrepareForHigh())
        {
            mGamePieceLocation = GamePieceLocation.HIGH;
        }
        else if(mControls.GetPrepareForMiddle())
        {
            mGamePieceLocation = GamePieceLocation.MID;
        }
        else if(mControls.GetPrepareForLow())
        {
            mGamePieceLocation = GamePieceLocation.LOW;
        }

        //for now, force manual mode
        mManualAimOverride = true;
        //for testing, pick an arbitary angle
        
        if(mControls.GetManualAimClockwise())
        {
            mManualAimDirection += 3.0;
            mManualAimOverride = true;
        }
        if(mControls.GetManualAimCounterClockwise())
        {
            mManualAimDirection -= 3.0;
            mManualAimOverride = true;
        }

        if(mManualAimDirection > 180)
        {
            mManualAimDirection = 180;
        }
        if(mManualAimDirection < -180)
        {
            mManualAimDirection = -180;
        }

        SmartDashboard.putNumber("mManualAimDirection", mManualAimDirection);
        double[] ntValues = new double[2];
        ntValues[0] = mManualAimOverride ? 1.0 : 0.0 ;
        ntValues[1] = mManualAimDirection;

        NetworkTableInstance.getDefault().getTable("limelight").putValue("llrobot", NetworkTableValue.makeDoubleArray(ntValues));
            
        
        
        //the only way to get out of intaking is to hit the score button.  Cool.
        if(mControls.GetInvokeScoringRoutine())
        {
            if(!mLastWasInvokingScoringRoutine)
            {
                mStartedInvokingScoringRoutineTime = System.currentTimeMillis();                
            }
            mLastWasInvokingScoringRoutine = true;

            if(System.currentTimeMillis() - mStartedInvokingScoringRoutineTime > SCORING_BUTTON_MIN_PRESS_TIME)
            {
                if(mControls.GetIsLowGoalMode())
                {
                    return new NextStateInfo(States.LowGoalScoring);

                }
                else
                {
                    return new NextStateInfo(States.Scoring, new ScoringInstructions(mGamePieceLocation, mControls.GetGamePieceMode(), mManualAimDirection, mManualAimOverride));
                }
            }
            else
            {
                return new NextStateInfo(States.Intaking); //keep on Intakin'
            }
        }
        else
        {
            mLastWasInvokingScoringRoutine = false;
            return new NextStateInfo(States.Intaking); //keep on Intakin'
        }

        

    }


    protected void EnterState(Object pEntryParameter)
    {
        super.EnterState(pEntryParameter);
        //For normal usage
        mIntake.SetMode(IntakeMode.NormalIntaking);
        //For now while testing:
        //mIntake.SetMode(IntakeMode.Holding);
        mManualAimOverride = false;
        mManualAimDirection = 0;
        
    }

    @Override
    protected States GetState() {
        return States.Intaking;
    }

    @Override
    protected String GetName() {
        
        return "Intaking";
    }
    
}
