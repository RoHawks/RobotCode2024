package states;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkFlex;

import frc.robot.Functionality;
import robosystems.ClimberArms;
import robosystems.ExtendoArm;
import robosystems.Intake;
import robosystems.Lights;
import robosystems.Lights.LightingScheme;
import robosystems.Shooter;
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

    private void basicContinousActions()
    {
        mSwerveDrive.Run(mControls);
        mExtendoArm.retract();
        mShooter.setAngleToIntakingAngle();
        mLights.SetLightingScheme(LightingScheme.Ejecting);
        mLights.Run();
        mShooterMode = Functionality.checkForShootingPreperationButtons(mControls, mShooterMode);
    }

    public NextStateInfo Run(){

        basicContinousActions();

        double timeToCoolDown = 250;
        if (GetTimeSinceEntry() < 2000 - timeToCoolDown)
        {
            mIntake.setToFirstEjectingSpeed();
            mShooter.setToFirstEjectingSpeed();
        }
        else if (GetTimeSinceEntry() < 2000 + timeToCoolDown)
        {
            mIntake.setToFirstEjectingSpeed();
            mShooter.setSpeed(0,0);
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
        return States.Ejecting;
    }

    public String GetName(){
        return "Ejecting";
    }



}