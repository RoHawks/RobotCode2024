package robotcode.autonomous.steps;

import edu.wpi.first.math.trajectory.ExponentialProfile.State;
import robosystems.Intake;
import robosystems.Shooter;
import robotcode.autonomous.AAction;
import states.JoystickControlsWithSwerve;
import states.States;

public class ShootAction extends AAction{

    private Shooter mShooter;
    private Intake mIntake;
    private boolean mIntakeRingsFromGround;

    public ShootAction(Shooter pShooter, Intake pIntake, boolean pIntakeRingsFromGround)
    {
        mShooter = pShooter;
        mIntake = pIntake;
        mIntakeRingsFromGround = pIntakeRingsFromGround;
    }

    @Override
    public boolean Run() 
    {
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
