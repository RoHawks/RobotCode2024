package states;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkFlex;

import robosystems.ClimberArms;
import robosystems.Intake;
import robosystems.Shooter;
import universalSwerve.SwerveDrive;

public class ClimbingState extends AState
{
    private ClimberArms mClimberArms;
    private ShooterMode mShooterMode;
    private SwerveDrive mSwerveDrive; 
    private Shooter mShooter;
    private Intake mIntake;

    public ClimbingState(SwerveDrive pSwerveDrive, ClimberArms pClimberArms, Shooter pShooter, Intake pIntake){
        mClimberArms = pClimberArms;
        mSwerveDrive = pSwerveDrive;
        mShooter = pShooter;
        mIntake = pIntake;
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
        mSwerveDrive.Run(ApproachingClimberControls.Instance);
        boolean mFinishedRetracting = mClimberArms.retract();
        mShooter.goToTrapShootAngle();
        mShooter.setSpeed(0, 0);
        mIntake.setToHoldingSpeed();

        if(mFinishedRetracting){
            return new NextStateInfo(States.TrapShooting, mShooterMode);
        }else{
            return new NextStateInfo(States.Climbing, mShooterMode);
        }
    }

    public States GetState(){
        return States.Climbing;
    }

    public String GetName(){
        return "Climbing";
    }



}