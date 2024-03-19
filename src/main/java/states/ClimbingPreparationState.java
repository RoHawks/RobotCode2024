package states;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkBase.ControlType;

import frc.robot.Constants;
import robosystems.ClimberArms;
import robosystems.Intake;
import robosystems.Shooter;
import states.ShooterMode;
import universalSwerve.SwerveDrive;

import com.revrobotics.CANSparkFlex;

public class ClimbingPreparationState extends AState
{
    private ClimberArms mClimberArms;
    private Shooter mShooter;
    private ShooterMode mShooterMode;
    private SwerveDrive mSwerveDrive;
    private Controls mControls;
    private Intake mIntake;

    public ClimbingPreparationState(SwerveDrive pSwerveDrive, ClimberArms pClimberArms, Shooter pShooter, Controls pControls, Intake pIntake){
        mSwerveDrive = pSwerveDrive;
        mClimberArms = pClimberArms;
        mShooter = pShooter;
        mControls = pControls;
        mIntake = pIntake;
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
        mSwerveDrive.Run(mControls);
        mShooter.setAngleToIntakingAngle();
        mShooter.setSpeed(0, 0);
        mIntake.setToHoldingSpeed();
        


        boolean mFinishedExtending = mClimberArms.extend();
        
        
        if (mControls.GetForceIntakingMode())
        {
            return new NextStateInfo(States.Intaking, mShooterMode);
        }
        
        if(mControls.GetRetractClimb()){ //remove mFinishedExtending && ... we can retratect at any time.
            return new NextStateInfo(States.Climbing, mShooterMode);
        }else{
            return new NextStateInfo(States.ClimbingPreparation, mShooterMode);
        }

    }

    public States GetState(){
        return States.ClimbingPreparation;
    }

    public String GetName(){
        return "ClimbingPreparation";
    }



}

