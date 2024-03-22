package robotcode.autonomous;

import java.util.ArrayList;

import edu.wpi.first.math.Pair;
import robosystems.Intake;
import robosystems.Shooter;
import robotcode.autonomous.steps.FinishAction;
import robotcode.autonomous.steps.FollowTrajectoryAction;
import robotcode.autonomous.steps.GoToShooterAngleAction;
import robotcode.autonomous.steps.IntakeThenHoldAction;
import robotcode.autonomous.steps.PrepShooterToShootAction;
import robotcode.autonomous.steps.ShootAction;
import robotcode.autonomous.steps.WaitAction;
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

    public Step CreateFollowPathAndWarmShooterStep(String pTrajectoryName, double pAngle)
    {
        Step returnValue = new Step("TurnWarmAndAngleShooterStep");
        returnValue.AddAction(new FollowTrajectoryAction(pTrajectoryName, mSwerveDrive, true, mIntake, false));        
        returnValue.AddAction(new PrepShooterToShootAction(mShooter, pAngle));
        return returnValue;
    }

    public Step CreateFollowPathThenIntakePieceAndHold(String pTrajectoryName, Boolean pSuckingInRings, ArrayList<Pair<Double, Long>> pPairsOfAnglesAndTimes)
    {
        Step returnValue = new Step("TurnWarmAndAngleShooterStep");
        returnValue.AddAction(new GoToShooterAngleAction(mShooter, pPairsOfAnglesAndTimes));
        returnValue.AddAction(new FollowTrajectoryAction(pTrajectoryName, mSwerveDrive, true, mIntake, pSuckingInRings));        
        returnValue.AddAction(new IntakeThenHoldAction(mShooter, mIntake));
        return returnValue;
    }


    public Step CreateShootStep()
    {
        Step returnValue = new Step("ShootStep");
        returnValue.AddAction(new ShootAction(mShooter, mIntake, false));        
        return returnValue;
    }
    
    public Step CreateWaitStep(long pTime)
    {
        Step returnValue = new Step("WaitStep");
        returnValue.AddAction(new WaitAction(pTime));        
        return returnValue;
    }

    public Step CreateFollowTrajectoryAndInstaShootStep(String pTrajectoryName, ArrayList<Pair<Double, Long>> pairsOfAnglesAndTimes)
    {
        Step returnValue = new Step("TrajectoryAndIntakeStep");
        
        returnValue.AddAction(new GoToShooterAngleAction(mShooter, pairsOfAnglesAndTimes));
        returnValue.AddAction(new FollowTrajectoryAction(pTrajectoryName, mSwerveDrive, false, mIntake, true));
        returnValue.AddAction(new ShootAction(mShooter, mIntake, true));    
        return returnValue;
    }

    public Step CreateFinishStep()
    {
        Step returnValue = new Step("FinishStep");
        returnValue.AddAction(new FinishAction(mSwerveDrive));
        return returnValue;
    }

}
