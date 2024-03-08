package states;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Functionality;
import robosystems.Shooter;
import robosystems.Lights.LightingScheme;
import robosystems.ExtendoArm;
import robosystems.Intake;
import robosystems.Lights;
import universalSwerve.SwerveDrive;



public class HoldingState extends AState {
    private SwerveDrive mSwerveDrive;
    private Intake mIntake;
    private Shooter mShooter;
    private ExtendoArm mExtendoArm;
    private Controls mControls;
    private Lights mLights;

    private HoldingMode mHoldingMode; // ARGH everything else is ShooterMode IntakeMode this naming convention DOESNT WORK for HoldingMode 
    private ShooterMode mShooterMode;

    public enum HoldingMode
    {
        BackingUp,
        Holding
    }
    
    public HoldingState(
        SwerveDrive pSwerveDrive,
        Intake pIntake,
        Shooter pShooter,
        ExtendoArm pExtendoArm,
        Controls pControls,
        Lights pLights
        )
        {
            mSwerveDrive = pSwerveDrive;
            mIntake = pIntake;
            mShooter = pShooter;
            mControls = pControls;   
            mExtendoArm = pExtendoArm;
            mHoldingMode = HoldingMode.Holding;
            mShooterMode = ShooterMode.HighGoalDriveBy;
            mLights = pLights;
        }
        
    protected void determineCurrentState()
    {
    
        if (mIntake.getBreakBeamStatus())
        {
            mHoldingMode = HoldingMode.BackingUp;
            return;
        }
        else 
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

    public void logHoldingStateValues()
    {
        SmartDashboard.putString("Holding State: HoldingMode", mHoldingMode.toString());
        SmartDashboard.putString("Holding State: ShooterMode", mShooterMode.toString());

    }


    private void basicContinousActions()
    {
        // logHoldingStateValues();
        mShooterMode = Functionality.checkForShootingPreperationButtons(mControls, mShooterMode);        
        mExtendoArm.retract();
        mShooter.checkIfPersistentlyHasCorrectSpeed(mShooterMode);
        // mShooter.logShooterInformation();
        mShooter.setAngleBasedOnShooterMode(mShooterMode);
        
        mLights.Run();
        determineCurrentState();

        if (mIntake.getBreakBeamStatus())
        {
            mIntake.recordPositionAtBreakBeam();
        }
    }

    @Override
    public NextStateInfo Run() {
        
        if (mShooterMode == ShooterMode.LowGoal)
        {
            mSwerveDrive.Run(mControls, true, Constants.LOW_GOAL_ROTATION);
            mLights.SetLightingScheme(LightingScheme.HoldingButNoCameraLock);
            
        }
        else if (mShooterMode == ShooterMode.HighGoalManual)
        {
            mSwerveDrive.Run(mControls);
            mLights.SetLightingScheme(LightingScheme.HoldingButNoCameraLock);

        }
        else if (mShooterMode == ShooterMode.AutoAim)
        {
            mSwerveDrive.Run(mControls); // later whatever stuff I need to do
            mLights.SetLightingScheme(LightingScheme.HoldingButNoCameraLock);

        } 
        else if (mShooterMode == ShooterMode.HighGoalDriveBy)
        {
            mSwerveDrive.Run(mControls, true, Constants.HIGH_GOAL_ROTATION);
        }
        
        basicContinousActions();

        
        if (mHoldingMode == HoldingMode.BackingUp)
        {
            mIntake.setConveyorToBackupSpeed();
        }
        else if (mHoldingMode == HoldingMode.Holding)
        {
            mIntake.setToHoldingSpeed();
            if (mShooterMode == ShooterMode.LowGoal)
            {
                mShooter.spinUpToLowGoalSpeed();
            }
            else
            {
                mShooter.spinUpToHighGoalSpeed();
            }
        }


        if (mControls.GetForceEjectionMode())
        {
            return new NextStateInfo(States.Ejecting, mShooterMode);
        }
        

        if(mControls.GetForceIntakingMode())
        {
            return new NextStateInfo(States.Intaking, mShooterMode);
        }




        if (mShooterMode == ShooterMode.HighGoalDriveBy && mHoldingMode == HoldingMode.Holding)
        {
            return new NextStateInfo(States.Shooting, ShooterMode.HighGoalDriveBy);
        }


        if (mControls.GetPrepareToClimb())
        {
            return new NextStateInfo(States.ClimbingPreparation, mShooterMode);
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

    protected void EnterState(Object pEntryParameter)
    {
        super.EnterState(pEntryParameter);
        //For normal usage
        mShooterMode = (ShooterMode) pEntryParameter;
        mShooter.resetForHoldingState();
        
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