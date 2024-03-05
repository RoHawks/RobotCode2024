package robotcode.autonomous;

import robosystems.Intake;
import robosystems.Shooter;
import universalSwerve.SwerveDrive;


public class RoutineFactory 
{
    private StepFactory mStepFactory;

    public RoutineFactory(SwerveDrive pSwerveDrive, Shooter pShooter, Intake pIntake)
    {
        mStepFactory = new StepFactory(pSwerveDrive, pShooter, pIntake);    }

    public AutonomousRoutine FourCloseRingAuto()
    {
        double instaShootAngle = 46;
        AutonomousRoutine returnValue = new AutonomousRoutine("FourCloseRingAuto");
        //returnValue.AddStep(mStepFactory.CreateGameStartStep());
        returnValue.AddStep(mStepFactory.CreateTurnWarmAndAngleShooterStep("4CloseRingAuto.1", instaShootAngle));
        returnValue.AddStep(mStepFactory.CreateShootStep());
        returnValue.AddStep(mStepFactory.CreateTrajectoryAndIntakeStep("4CloseRingAuto.2"));
        returnValue.AddStep(mStepFactory.CreateFinishStep());
        
        return returnValue;

    }
    

}
