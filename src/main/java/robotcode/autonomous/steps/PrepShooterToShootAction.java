package robotcode.autonomous.steps;

import robosystems.Shooter;
import robotcode.autonomous.AAction;
import states.JoystickControlsWithSwerve;
import states.ShooterMode;

public class PrepShooterToShootAction extends AAction
{

    private double mAngle;
    private Shooter mShooter;

    public PrepShooterToShootAction(Shooter pShooter, double pAngle)
    {
        mShooter = pShooter;
        mAngle = pAngle;
    }

    @Override
    public boolean Run() 
    {
        mShooter.setAngle(mAngle);    
        mShooter.spinUpToHighGoalSpeed();

        return
            mShooter.checkIfPersistentlyHasCorrectSpeed(ShooterMode.HighGoalManual)
            && mShooter.checkIfAnglerIsCloseEnough();

        
    }

    @Override
    public void EndAction() {
        
    }
    
    
}
