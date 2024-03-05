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

    public ShootAction(Shooter pShooter, Intake pIntake)
    {
        mShooter = pShooter;
        mIntake = pIntake;
    }

    @Override
    public boolean Run() 
    {
        //mIntake.setToLaunchingNoteIntoTheShooterSpeed();
        if(HasTimeElapsed(1000))
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
