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



public class IntakingState extends AState {
    private SwerveDrive mSwerveDrive;
    private Intake mIntake;
    private Shooter mShooter;
    private ExtendoArm mExtendoArm;
    private Controls mControls;
    private Lights mLights;


    private ShooterMode mShooterMode;


    public IntakingState(
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
            mExtendoArm = pExtendoArm;
            mControls = pControls;   
            mShooterMode = ShooterMode.HighGoalDriveBy;
            mLights = pLights;
        }
    
    
    public void logIntakingStateValues()
    {
        SmartDashboard.putString("Intaking State: ShooterMode", mShooterMode.toString());
    }
    

    private void basicContinousActions()
    {
        mSwerveDrive.Run(mControls);
        mShooter.setAngleToIntakingAngle();
        mShooter.setSpeed(0,0);
        mExtendoArm.retract();
        mLights.SetLightingScheme(LightingScheme.Intaking);
        mLights.Run();
        mShooterMode = Functionality.checkForShootingPreperationButtons(mControls, mShooterMode);
    }

    /**
     * Runs the intake. <br><br>
     * Ejection will only work while the button is held, and will go back to normal intaking if released
     */
    @Override
    public NextStateInfo Run() {
        // ATS commented out for tests!

        basicContinousActions();

        
        if (GetTimeSinceEntry() < 400)
        {
            mIntake.setToHoldingSpeed();
        }
        else
        {
            mIntake.setToNormalIntakingSpeed();
        } 
        
        
        if (GetTimeSinceEntry() < 1000)
        {
            mShooter.setSpeed(0, 0);
            
        }
        else
        {
            mShooter.backingUpNoteToPreventFallingOut();
        }

        

        if (mControls.GetForceEjectionMode())
        {
            return new NextStateInfo(States.Ejecting, mShooterMode);
        }
        else if (mControls.GetPrepareToClimb())
        {
            return new NextStateInfo(States.ClimbingPreparation, mShooterMode);
        }
        else if (mIntake.getBreakBeamStatus())
        {
            mIntake.recordPositionAtBreakBeam();
            return new NextStateInfo(States.Holding, mShooterMode);
        }
        else
        {
            return new NextStateInfo(States.Intaking, mShooterMode);
        }
    }



    protected void EnterState(Object pEntryParameter)
    {
        super.EnterState(pEntryParameter);
        //For normal usage

        mLights.SetLightingScheme(LightingScheme.Intaking);
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