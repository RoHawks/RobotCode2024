package states;

import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;

import robosystems.Shooter;
import states.IntakingState.IntakeMode;
import robosystems.Intake;

import universalSwerve.SwerveDrive;



public class ShootingState extends AState {
    private SwerveDrive mSwerveDrive;
    private Intake mIntake;
    private Shooter mShooter;
    private Controls mControls;

    private ShooterMode mShooterMode;

    public ShootingState(
        SwerveDrive pSwerveDrive,
        Intake pIntake,
        Shooter pShooter,
        Controls pControls
        )
        {
            mSwerveDrive = pSwerveDrive;
            mIntake = pIntake;
            mShooter = pShooter;
            mControls = pControls;   

            mShooterMode = ShooterMode.Undefined;
        }
        
    /**
     * Runs the intake. <br><br>
     * Ejection will only work while the button is held, and will go back to normal intaking if released
     */
    @Override
    protected NextStateInfo Run() {
        mSwerveDrive.Run(mControls);
        mShooter.setAngleBasedOnShooterMode(mShooterMode);

        if (mShooterMode == ShooterMode.LowGoal || mShooterMode == ShooterMode.HighGoalManual)
        {
            mShooter.shootAtDefaultSpeed();
        }
        else if (mShooterMode == ShooterMode.AutoAim)
        {

        } 
        else if (mShooterMode == ShooterMode.HighGoalDriveBy)
        {

        }

        SmartDashboard.putString("ShooterMode", mShooterMode.name());

        
    
        if(mControls.GetForceIntakingMode())
        {
            return new NextStateInfo(States.Intaking);
        }

        if (this.GetTimeSinceEntry() > 3000)
        {
            return new NextStateInfo(States.Intaking, mShooterMode);
        }
        else
        {
            return new NextStateInfo(States.Shooting, mShooterMode);
        }
        
    }

  

    protected void EnterState(Object pEntryParameter)
    {
        super.EnterState(pEntryParameter);
        //For normal usage
        mShooterMode = (ShooterMode) pEntryParameter;
        
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