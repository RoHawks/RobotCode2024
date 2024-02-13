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
        
    protected void determineCurrentState()
    {
        if (mControls.GetForceEjectionMode())
        {
            mHoldingMode = HoldingMode.Ejecting;
        }
        else
        {
            if (mIntake.getBreakBeamStatus())
            {
                mHoldingMode = HoldingMode.BackingUp;
                return;
            }
            if (mHoldingMode != HoldingMode.Holding)
            {
                boolean hasFinishedBackingUp = mIntake.hasConveyorFinishedBackingUp();
                if (hasFinishedBackingUp) 
                {
                    mHoldingMode = HoldingMode.Holding;
                }
                else
                {
                    mHoldingMode = HoldingMode.BackingUp;
                }
            }
            
        }
    }

    public void logHoldingStateValues()
    {
        SmartDashboard.putString("Holding State: HoldingMode", mHoldingMode.toString());
        SmartDashboard.putString("Holding State: ShooterMode", mShooterMode.toString());
    }

    @Override
    public NextStateInfo Run() {
        // ATS commented out for tests!
        //mSwerveDrive.Run(mControls);
        mShooter.setAngleBasedOnShooterMode(mShooterMode);
        mShooter.shootAtDefaultSpeed();
        determineCurrentState();

        if (mIntake.getBreakBeamStatus())
        {
            mIntake.recordPositionAtBreakBeam();
        }

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