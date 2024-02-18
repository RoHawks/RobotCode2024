package states;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

import robosystems.Shooter;
import states.IntakingState.IntakeMode;
import robosystems.ExtendoArm;
import robosystems.Intake;

import universalSwerve.SwerveDrive;



public class ShootingState extends AState {
    private SwerveDrive mSwerveDrive;
    private Intake mIntake;
    private Shooter mShooter;
    private ExtendoArm mExtendoArm;
    private Controls mControls;

    private ShooterMode mShooterMode;
    private double mTimeStartedToShoot;

    private boolean mHasShot;

    public ShootingState(
        SwerveDrive pSwerveDrive,
        Intake pIntake,
        Shooter pShooter,
        ExtendoArm pExtendoArm,
        Controls pControls
        )
        {
            mSwerveDrive = pSwerveDrive;
            mIntake = pIntake;
            mShooter = pShooter;
            mControls = pControls;   
            mExtendoArm = pExtendoArm;
            mShooterMode = ShooterMode.HighGoalDriveBy;
            mTimeStartedToShoot = -1;
            mHasShot = false;
        }
        
    /**
     * Runs the intake. <br><br>
     * Ejection will only work while the button is held, and will go back to normal intaking if released
     */
    @Override
    public NextStateInfo Run() {
        // ATS commented out for tests!
        mSwerveDrive.Run(mControls);
        logShootingStateInformation();
        checkForShootingPreperationButtons();
        mShooter.setAngleBasedOnShooterMode(mShooterMode);
        
        mShooter.spinUpToCorrectSpeed();

        if (mShooterMode == ShooterMode.LowGoal)
        {
            mExtendoArm.lowGoalExtension();
        }
        else if (mShooterMode == ShooterMode.HighGoalManual)
        {
            mExtendoArm.retract();
        }
        else if (mShooterMode == ShooterMode.AutoAim)
        {
            mExtendoArm.retract();
        } 
        else if (mShooterMode == ShooterMode.HighGoalDriveBy)
        {
            mExtendoArm.retract();
        }

        boolean inLowGoalAndExtendedFarEnough = !(mShooterMode == ShooterMode.LowGoal && !mExtendoArm.hasReachedLowGoal());
        
        if (mShooter.checkIfPersistentlyHasCorrectSpeed()
            && inLowGoalAndExtendedFarEnough)
        {
            if (!mHasShot)
            {
                mHasShot = true;
                mTimeStartedToShoot = System.currentTimeMillis();
            } 
            mIntake.setToLaunchingNoteIntoTheShooterSpeed();
        }
        else
        {
            mIntake.setToHoldingSpeed();
        }
        
        SmartDashboard.putString("ShooterMode", mShooterMode.name());
        
        
    
        if(mControls.GetForceIntakingMode())
        {
            return new NextStateInfo(States.Intaking, mShooterMode);
        }

        SmartDashboard.putNumber("Shooting: Time Elapsed Since Shooting", System.currentTimeMillis() - mTimeStartedToShoot);

        if (mHasShot && System.currentTimeMillis() - mTimeStartedToShoot > 3000)
        {
            return new NextStateInfo(States.Intaking, mShooterMode);
        }
        else
        {
            return new NextStateInfo(States.Shooting, mShooterMode);
        }
        
    }

    private void checkForShootingPreperationButtons()
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

    public void logShootingStateInformation()
    {
        SmartDashboard.putString("Shooting: Shooting Mode", mShooterMode.name());
        SmartDashboard.putBoolean("Persistently at Speed", mShooter.checkIfPersistentlyHasCorrectSpeed());
        mShooter.logShooterInformation();
    }

  

    protected void EnterState(Object pEntryParameter)
    {
        super.EnterState(pEntryParameter);
        //For normal usage
        mShooterMode = (ShooterMode) pEntryParameter;
        mTimeStartedToShoot = -1;
        
        mHasShot = false;
    }

    

    @Override
    protected States GetState() {
        return States.Shooting;
    }

    @Override
    protected String GetName() {
        return "Shooting";
    }
    
}