package states;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

import robosystems.Shooter;
import robosystems.Intake;

import universalSwerve.SwerveDrive;



public class HoldingState extends AState {
    private SwerveDrive mSwerveDrive;
    private Intake mIntake;
    private Shooter mShooter;
    private Controls mControls;

    private HoldingMode mHoldingMode; // ARGH everything else is ShooterMode IntakeMode this naming convention DOESNT WORK for HoldingMode 
    private ShooterMode mShooterMode;

    private long mTimeEnteredIntakeMode;

    public enum HoldingMode
    {
        BackingUp,
        Holding,
        Ejecting
    }

    public HoldingState(
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
            mHoldingMode = HoldingMode.Holding;
            mShooterMode = ShooterMode.Undefined;
        }
        
    protected void determineCurrentStateAndRecordConveyorPositionAtTop()
    {
        if (mControls.GetForceEjectionMode())
        {
            mHoldingMode = HoldingMode.Ejecting;
        }
        else
        {
            if (mControls.IsPieceInIntake())
            {
                mHoldingMode = HoldingMode.BackingUp;
                mIntake.recordPositionAtBreakBeam();
            }

            if (mHoldingMode == HoldingMode.BackingUp)
            {
                boolean hasFinishedBackingUp = mIntake.hasConveyorFinishedBackingUp();
                if (hasFinishedBackingUp) 
                {
                    mHoldingMode = HoldingMode.Holding;
                }
            }
        }
    }

    @Override
    protected NextStateInfo Run() {
        mSwerveDrive.Run(mControls);
        mShooter.setAngleBasedOnShooterMode(mShooterMode);

        
        determineCurrentStateAndRecordConveyorPositionAtTop();

        if (mHoldingMode == HoldingMode.BackingUp)
        {
            mIntake.setConveyorToBackupSpeed();
        }
        else if (mHoldingMode == HoldingMode.Holding)
        {
            mIntake.setToHoldingSpeed();
        }
        else if (mHoldingMode == HoldingMode.Ejecting)
        {
            mIntake.setToEjectingSpeed();
        } 

        
        checkForShootingPreperationButtons();
        mShooter.setAngleBasedOnShooterMode(mShooterMode);


        if(mControls.GetForceIntakingMode())
        {
            return new NextStateInfo(States.Intaking);
        }

        if (mControls.GetStartShootingSequence())
        {
            return new NextStateInfo(States.Shooting, mShooterMode);
        }
        else
        {
            return new NextStateInfo(States.Holding, mShooterMode);
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


    protected void EnterState(Object pEntryParameter)
    {
        super.EnterState(pEntryParameter);
        //For normal usage
        mShooterMode = (ShooterMode) pEntryParameter;
        
    }

    @Override
    protected States GetState() {
        return States.Holding;
    }

    @Override
    protected String GetName() {
        return "Holding";
    }
    
}