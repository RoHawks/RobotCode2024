package states;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

import robosystems.Shooter;
import robosystems.ExtendoArm;
import robosystems.Intake;

import universalSwerve.SwerveDrive;



public class IntakingState extends AState {
    private SwerveDrive mSwerveDrive;
    private Intake mIntake;
    private Shooter mShooter;
    private ExtendoArm mExtendoArm;
    private Controls mControls;

    private IntakeMode mIntakeMode;
    private ShooterMode mShooterMode;

    public enum IntakeMode
    {
        NormalIntaking,
        Ejecting
    }

    public IntakingState(
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
            mExtendoArm = pExtendoArm;
            mControls = pControls;   
            mIntakeMode = IntakeMode.NormalIntaking;
            mShooterMode = ShooterMode.HighGoalDriveBy;
        }
    
    
    public void logIntakingStateValues()
    {
        SmartDashboard.putString("Intaking State: IntakingMode", mIntakeMode.toString());
        SmartDashboard.putString("Intaking State: ShooterMode", mShooterMode.toString());
    }
    
    /**
     * Runs the intake. <br><br>
     * Ejection will only work while the button is held, and will go back to normal intaking if released
     */
    @Override
    public NextStateInfo Run() {
        // ATS commented out for tests!
        mSwerveDrive.Run(mControls);
        logIntakingStateValues();
        mShooter.setAngleToIntakingAngle();
        mShooter.setSpeed(0,0);
        mExtendoArm.retract();
        
        if (mIntakeMode == IntakeMode.NormalIntaking)
        {
            mIntake.setToNormalIntakingSpeed();
        }
        else if (mIntakeMode == IntakeMode.Ejecting)
        {
            mIntake.setToEjectingSpeed();
        } 
        
        
        if (mControls.GetForceEjectionMode())
        {
            setIntakingMode(IntakeMode.Ejecting);
        }
        else
        {
            setIntakingMode(IntakeMode.NormalIntaking);
        }

        if(mControls.GetForceIntakingMode())
        {
            setIntakingMode(IntakeMode.NormalIntaking);
        }
                
        checkForShootingPreperationButtons();
        

        if (mIntake.getBreakBeamStatus())
        {
            mIntake.recordPositionAtBreakBeam();
            return new NextStateInfo(States.Holding, mShooterMode);
        }
        else
        {
            return new NextStateInfo(States.Intaking, mShooterMode);
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
    }


    protected void EnterState(Object pEntryParameter)
    {
        super.EnterState(pEntryParameter);
        //For normal usage
        setIntakingMode(IntakeMode.NormalIntaking);
        if (pEntryParameter instanceof ShooterMode)
        {
            mShooterMode = (ShooterMode) pEntryParameter;
        }
        
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