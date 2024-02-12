package states;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

import robosystems.Shooter;
import robosystems.Intake;

import universalSwerve.SwerveDrive;



public class IntakingState extends AState {
    private SwerveDrive mSwerveDrive;
    private Intake mIntake;
    private Shooter mShooter;
    private Controls mControls;

    private IntakeMode mIntakeMode;
    private ShooterMode mShooterMode;

    private long mTimeEnteredIntakeMode;

    public enum IntakeMode
    {
        NormalIntaking,
        Ejecting
    }

    public IntakingState(
        SwerveDrive pSwerveDrive,
        Intake pIntake,
        Shooter pShooter,
        Controls pControls
        )
        {
            mSwerveDrive = pSwerveDrive;
            mIntake = pIntake;
            mShooter = pShooter;
            mControls = pControls;   
            mIntakeMode = IntakeMode.NormalIntaking;
            mShooterMode = ShooterMode.Undefined;
        }
        
    /**
     * Runs the intake. <br><br>
     * Ejection will only work while the button is held, and will go back to normal intaking if released
     */
    @Override
    protected NextStateInfo Run() {
        mSwerveDrive.Run(mControls);
        mShooter.setAngleToIntakingAngle();

        if (mIntakeMode == IntakeMode.NormalIntaking)
        {
            mIntake.setToNormalIntakingSpeed();
        }
        else if (mIntakeMode == IntakeMode.Ejecting)
        {
            mIntake.setToEjectingSpeed();
        } 

        if(mControls.GetForceIntakingMode())
        {
            setIntakingMode(IntakeMode.NormalIntaking);
        }
        
        
        if(mControls.GetForceEjectionMode())
        {
            setIntakingMode(IntakeMode.Ejecting);
        }
        else
        {
            setIntakingMode(IntakeMode.NormalIntaking);
        }
                
        checkForShootingPreperationButtons();
        

        if (mControls.IsPieceInIntake())
        {
            mIntake.recordPositionAtBreakBeam();
            return new NextStateInfo(States.Holding, mShooterMode);
        }
        else
        {
            return new NextStateInfo(States.Intaking);
        }
    }

    public void checkForShootingPreperationButtons()
    {
        if(mControls.GetPrepareForHighGoalManual())
        {
            mShooterMode = ShooterMode.HighGoalManual;
        }
        else if(mControls.GetPrepareForHighGoalDriveBy())
        {
            mShooterMode = ShooterMode.HighGoalDriveBy;
        }
        else if(mControls.GetPrepareForLowGoal())
        {
            mShooterMode = ShooterMode.LowGoal;
        }
        else if(mControls.GetPrepareForAutoAim())
        {
            mShooterMode = ShooterMode.AutoAim;
        }
    }

    private void setIntakingMode(IntakeMode pIntakeMode)
    {
        mIntakeMode = pIntakeMode;
        mTimeEnteredIntakeMode = System.currentTimeMillis();
    }


    protected void EnterState(Object pEntryParameter)
    {
        super.EnterState(pEntryParameter);
        //For normal usage
        setIntakingMode(IntakeMode.NormalIntaking);
        
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