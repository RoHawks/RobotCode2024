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
        ApproachingClimberControls.Instance.setTimeSinceEnteredClimbingState(System.currentTimeMillis());
        mShooterMode = (ShooterMode) pEntryParameter;
        mControls.TurnOffVibrate();
    }

    public void ExitState()
    {

    }

    public NextStateInfo Run(){
        mSwerveDrive.Run(ApproachingClimberControls.Instance);
        mShooter.goToTrapShootAngle();
        mShooter.setSpeed(0, 0);
        mIntake.setToHoldingSpeed();
        


        boolean mFinishedExtending = mClimberArms.extend();
        
        if(mFinishedExtending && mControls.GetRetractClimb()){
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

