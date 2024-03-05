package states;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkFlex;

import robosystems.ClimberArms;
import robosystems.ExtendoArm;
import robosystems.Intake;
import robosystems.Lights;
import robosystems.Shooter;
import states.IntakingState.IntakeMode;
import universalSwerve.SwerveDrive;

public class EjectingState extends AState
{
    private SwerveDrive mSwerveDrive;
    private Intake mIntake;
    private Shooter mShooter;
    private ExtendoArm mExtendoArm;
    private Controls mControls;
    private Lights mLights;

    private ShooterMode mShooterMode;

    public EjectingState(
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
        mLights = pLights;
    }


    public void EnterState(Object pEntryParameter)
    {
        super.EnterState(pEntryParameter);
        mShooterMode = (ShooterMode) pEntryParameter;
    }

    public void ExitState()
    {

    }

    public NextStateInfo Run(){

        mSwerveDrive.Run(mControls);
        
        mExtendoArm.retract();


        if (GetTimeSinceEntry() < 2000)
        {
            mIntake.setToFirstEjectingSpeed();
            mShooter.backingUpNoteToPreventFallingOut();
        }
        else if (GetTimeSinceEntry() < 4000)
        {
            mIntake.setToSecondEjectingSpeed();
            mShooter.setToSecondEjectingSpeed();
        }
        else
        {
            return new NextStateInfo(States.Intaking, mShooterMode);
        }




        if (mControls.GetForceIntakingMode())
        {
            return new NextStateInfo(States.Intaking, mShooterMode);
        }
        return new NextStateInfo(States.Ejecting, mShooterMode);

    }

    public States GetState(){
        return States.TrapShooting;
    }

    public String GetName(){
        return "TrapShooting";
    }



}