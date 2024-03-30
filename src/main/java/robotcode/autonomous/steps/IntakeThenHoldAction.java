package robotcode.autonomous.steps;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import robosystems.Intake;
import robosystems.Shooter;
import robotcode.autonomous.AAction;
import states.HoldingState.HoldingMode;
import states.JoystickControlsWithSwerve;
import states.ShooterMode;

public class IntakeThenHoldAction extends AAction
{

    private double mAngle;
    private Shooter mShooter;
    private Intake mIntake;
    private HoldingMode mHoldingMode;
    private boolean mHasBeenInBackupMode;
    private boolean mFinishedBackingUp;

    public IntakeThenHoldAction(Shooter pShooter, Intake pIntake)
    {
        mShooter = pShooter;
        mIntake = pIntake;
        mHoldingMode = HoldingMode.Intaking;
        mHasBeenInBackupMode = false;
        mFinishedBackingUp = false;
    }

    public enum HoldingMode
    {
        BackingUp,
        Holding,
        Intaking
    }

    private void determineCurrentState()
    {
        if (mFinishedBackingUp)
        {
            mHoldingMode = HoldingMode.Holding;
            return;
        }

        if (mIntake.getBreakBeamStatus())
        {
            mHasBeenInBackupMode = true;
            mHoldingMode = HoldingMode.BackingUp;
            return;
        }
        else if (mHoldingMode != HoldingMode.Intaking)
        {
            boolean hasFinishedBackingUp = mIntake.hasConveyorFinishedBackingUp();
            if (hasFinishedBackingUp) 
            {
                mHoldingMode = HoldingMode.Holding;
                mFinishedBackingUp = true;
            }
            else
            {
                mHoldingMode = HoldingMode.BackingUp;
            }
        }
        
    }

    @Override
    public boolean Run() 
    {
        determineCurrentState();
        if (mIntake.getBreakBeamStatus())
        {
            mIntake.recordPositionAtBreakBeam();
        }
        SmartDashboard.putString("Auto Holding Mode State: ", mHoldingMode.name());
        if (mHoldingMode == HoldingMode.BackingUp)
        {
            mShooter.setSpeed(0,0);
            mIntake.setConveyorToBackupSpeed();
        }
        else if (mHoldingMode == HoldingMode.Holding)
        {
            mShooter.spinUpToHighGoalSpeed();
            mIntake.setToHoldingSpeed();
        }
        else if (mHoldingMode == HoldingMode.Intaking)
        {
            mShooter.backingUpNoteToPreventFallingOut();
            mIntake.setToNormalIntakingSpeed();
        }
        
 
        return true;
        
    }

    @Override
    public void EndAction() {
        
    }
    
    
}
