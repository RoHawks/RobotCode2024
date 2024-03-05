package robotcode.autonomous;

import robosystems.Intake;
import robosystems.Shooter;
import robotcode.autonomous.steps.FinishAction;
import robotcode.autonomous.steps.FollowTrajectoryAction;
import robotcode.autonomous.steps.PrepShooterToShootAction;
import robotcode.autonomous.steps.ShootAction;
import universalSwerve.SwerveDrive;

public class StepFactory 
{
    private SwerveDrive mSwerveDrive;
    private Shooter mShooter;
    private Intake mIntake;
    public StepFactory(SwerveDrive pSwerveDrive, Shooter pShooter, Intake pIntake)
    {
        mSwerveDrive = pSwerveDrive;
        mShooter = pShooter;
        mIntake = pIntake;
    }

    public Step CreateTurnWarmAndAngleShooterStep(String pTrajectoryName, double pAngle)
    {
        Step returnValue = new Step("TurnWarmAndAngleShooterStep");
        returnValue.AddAction(new FollowTrajectoryAction(pTrajectoryName, mSwerveDrive, true, mIntake, false));        
        returnValue.AddAction(new PrepShooterToShootAction(mShooter, pAngle));
        return returnValue;

    }

    public Step CreateShootStep()
    {
        Step returnValue = new Step("ShootStep");
        returnValue.AddAction(new ShootAction(mShooter, mIntake));        
        return returnValue;
    }

    public Step CreateTrajectoryAndIntakeStep(String pTrajectoryName)
    {
        Step returnValue = new Step("TrajectoryAndIntakeStep");
        returnValue.AddAction(new FollowTrajectoryAction(pTrajectoryName, mSwerveDrive, false, mIntake, true));
        returnValue.AddAction(new ShootAction(mShooter, mIntake));    
        return returnValue;
    }

    public Step CreateFinishStep()
    {
        Step returnValue = new Step("TrajectoryAndIntakeStep");
        returnValue.AddAction(new FinishAction(mSwerveDrive));
        return returnValue;
    }

}
