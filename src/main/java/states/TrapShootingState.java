package states;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkFlex;

import robosystems.ClimberArms;
import robosystems.Shooter;
import universalSwerve.SwerveDrive;

public class TrapShootingState extends AState
{
    private Controls mControls;
    private ShooterMode mShooterMode;

    public TrapShootingState(Controls pControls){
        mControls = pControls;
    }

    public void EnterState(Object pEntryParameter)
    {
        super.EnterState(pEntryParameter);
        mShooterMode = (ShooterMode) pEntryParameter;
        mControls.TurnOffVibrate();
        
    }

    public void ExitState()
    {

    }

    public NextStateInfo Run(){

        if (mControls.GetForceIntakingMode())
        {
            return new NextStateInfo(States.Intaking, mShooterMode);
        }
        return new NextStateInfo(States.TrapShooting, mShooterMode);

    }

    public States GetState(){
        return States.TrapShooting;
    }

    public String GetName(){
        return "TrapShooting";
    }



}