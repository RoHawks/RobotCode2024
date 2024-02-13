package states;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

import robosystems.Shooter;
import states.IntakingState.IntakeMode;
import robosystems.Intake;

import universalSwerve.SwerveDrive;



public class ShootingState extends AState {
    private SwerveDrive mSwerveDrive;
    private Intake mIntake;
    private Shooter mShooter;
    private Controls mControls;

    private ShooterMode mShooterMode;

    public ShootingState(
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

            mShooterMode = ShooterMode.Undefined;
        }
        
    /**
     * Runs the intake. <br><br>
     * Ejection will only work while the button is held, and will go back to normal intaking if released
     */
    @Override
    public NextStateInfo Run() {
        // ATS commented out for tests!
        // mSwerveDrive.Run(mControls);
        mShooter.setAngleBasedOnShooterMode(mShooterMode);
        mIntake.setToNormalIntakingSpeed();
        if (mShooterMode == ShooterMode.LowGoal || mShooterMode == ShooterMode.HighGoalManual)
        {
            mShooter.shootAtDefaultSpeed();
        }
        else if (mShooterMode == ShooterMode.AutoAim)
        {

        } 
        else if (mShooterMode == ShooterMode.HighGoalDriveBy)
        {

        }

        SmartDashboard.putString("ShooterMode", mShooterMode.name());
        checkForShootingPreperationButtons();
        
    
        if(mControls.GetForceIntakingMode())
        {
            return new NextStateInfo(States.Intaking);
        }

        if (this.GetTimeSinceEntry() > 3000)
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

  

    protected void EnterState(Object pEntryParameter)
    {
        super.EnterState(pEntryParameter);
        //For normal usage
        mShooterMode = (ShooterMode) pEntryParameter;
        
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