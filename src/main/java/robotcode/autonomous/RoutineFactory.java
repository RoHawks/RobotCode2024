package robotcode.autonomous;

import java.util.ArrayList;

import edu.wpi.first.math.Pair;
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
        double initialAngle = 36;
        double instaShootAngle = 25;
        AutonomousRoutine returnValue = new AutonomousRoutine("FourCloseRingAuto");
        //returnValue.AddStep(mStepFactory.CreateGameStartStep());
        returnValue.AddStep(mStepFactory.CreateFollowPathAndWarmShooterStep("4CloseRingAuto.1", initialAngle));
        returnValue.AddStep(mStepFactory.CreateShootStep());

        ArrayList<Pair<Double,Long>> listOfPairs = new ArrayList<>();
        listOfPairs.add(new Pair<Double,Long>(27.0, 1250l));
        listOfPairs.add(new Pair<Double,Long>(30.0, 4000l));
        listOfPairs.add(new Pair<Double,Long>(18.0, 100000l));
    

        returnValue.AddStep(mStepFactory.CreateFollowTrajectoryAndInstaShootStep("4CloseRingAuto.2", listOfPairs));
        returnValue.AddStep(mStepFactory.CreateFinishStep());
        
        return returnValue;

    }

    public AutonomousRoutine OnLeftTwoNotePlusMidfield()
    {
        double initialAngle = 36;
        AutonomousRoutine returnValue = new AutonomousRoutine("OnLeftTwoNotePlusMidfield");
        //returnValue.AddStep(mStepFactory.CreateGameStartStep());
        returnValue.AddStep(mStepFactory.CreateFollowPathAndWarmShooterStep("OnLeftTwoNotePlusMidfield.1", initialAngle));
        returnValue.AddStep(mStepFactory.CreateShootStep());

        ArrayList<Pair<Double,Long>> eightTeenDegrees = new ArrayList<>();
        eightTeenDegrees.add(new Pair<Double,Long>(18.0, 100000l));
        returnValue.AddStep(mStepFactory.CreateFollowTrajectoryAndInstaShootStep("OnLeftTwoNotePlusMidfield.2", eightTeenDegrees));
        
        ArrayList<Pair<Double,Long>> thirtySixDegrees = new ArrayList<>();
        thirtySixDegrees.add(new Pair<Double,Long>(36.0, 100000l));
        
        returnValue.AddStep(mStepFactory.CreateFollowPathThenIntakePieceAndHold("OnLeftTwoNotePlusMidfield.3", null, thirtySixDegrees));
        returnValue.AddStep(mStepFactory.CreateShootStep());
        returnValue.AddStep(mStepFactory.CreateFollowPathThenIntakePieceAndHold("OnLeftTwoNotePlusMidfield.4", null, eightTeenDegrees));
        returnValue.AddStep(mStepFactory.CreateFinishStep());
        
        return returnValue;

    }
    

}
