package robotcode.autonomous.steps;

import edu.wpi.first.math.trajectory.ExponentialProfile.State;
import robosystems.Intake;
import robosystems.Shooter;
import robotcode.autonomous.AAction;
import states.JoystickControlsWithSwerve;
import states.States;
import universalSwerve.SwerveDrive;

public class ShootAction extends AAction{

    private Shooter mShooter;
    private Intake mIntake;
    private boolean mIntakeRingsFromGround;
    private SwerveDrive mSwerveDrive;

    public ShootAction(Shooter pShooter, Intake pIntake, boolean pIntakeRingsFromGround, SwerveDrive pSwerveDrive)
    {
        mShooter = pShooter;
        mIntake = pIntake;
        mIntakeRingsFromGround = pIntakeRingsFromGround;
        mSwerveDrive = pSwerveDrive;
    }

    @Override
    public boolean Run() 
    {
        // mSwerveDrive.StopTranslationButAllowWheelDirection();
        mSwerveDrive.UpdateOdometry();
        
        if(mIntakeRingsFromGround)
        {
            mIntake.setToInstaLaunch();
        }
        else
        {
            mIntake.setToLaunchingNoteIntoTheShooterSpeed();
        }
        if(HasTimeElapsed(2000))
        {
            return true;
        }
        else
        {
            return false;
        }
    }

    @Override
    public void EndAction() {
        
    }
    
    
}
