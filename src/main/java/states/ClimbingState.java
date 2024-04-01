package states;

import com.ctre.phoenix6.hardware.TalonFX;
import com.revrobotics.CANSparkFlex;

import frc.robot.Functionality;
import robosystems.ClimberArms;
import robosystems.Intake;
import robosystems.Shooter;
import states.ClimbingModeManager.ClimbingMode;
import universalSwerve.SwerveDrive;

public class ClimbingState extends AState
{
    private ClimberArms mClimberArms;
    private ShooterMode mShooterMode;
    private SwerveDrive mSwerveDrive; 
    private Shooter mShooter;
    private Intake mIntake;
    private Controls mControls;

    public ClimbingState(SwerveDrive pSwerveDrive, ClimberArms pClimberArms, Shooter pShooter, Intake pIntake, Controls pControls){
        mClimberArms = pClimberArms;
        mSwerveDrive = pSwerveDrive;
        mShooter = pShooter;
        mIntake = pIntake;
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

    public NextStateInfo Run()
    {
        mSwerveDrive.Run(mControls);
        boolean mFinishedRetracting = mClimberArms.retract();
        mShooter.setAngleToIntakingAngle();
        mShooter.setSpeed(0, 0);
        mIntake.setToHoldingSpeed();

        if (mControls.GetForceIntakingMode() && mFinishedRetracting)
        {
            ClimbingModeManager.setClimbingMode(ClimbingMode.Off);
            return new NextStateInfo(States.Intaking, mShooterMode);
        }

        ClimbingModeManager.determineClimbingMode(mControls);
        if (mControls.GetPrepareToClimb())
        {
            ClimbingModeManager.setClimbingMode(ClimbingMode.Extending);
            return new NextStateInfo(States.Intaking, mShooterMode);
        }

        if(mFinishedRetracting){
            // Do nothing!
            return new NextStateInfo(States.Climbing, mShooterMode); 
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