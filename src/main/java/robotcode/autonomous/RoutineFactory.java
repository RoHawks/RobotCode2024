package robotcode.autonomous;

import java.util.ArrayList;

import edu.wpi.first.math.Pair;
import robosystems.Intake;
import robosystems.Shooter;
import robotcode.autonomous.steps.WaitAction;
import universalSwerve.SwerveDrive;


public class RoutineFactory 
{
    private StepFactory mStepFactory;

    ArrayList<Pair<Double,Long>> unknownAmpSideAngle = new ArrayList<>();
    static{


    }
        

    public RoutineFactory(SwerveDrive pSwerveDrive, Shooter pShooter, Intake pIntake)
    {
        mStepFactory = new StepFactory(pSwerveDrive, pShooter, pIntake);    }

    public AutonomousRoutine SimpleTestPath()
    {
        AutonomousRoutine returnValue = new AutonomousRoutine("SimpleTestPath");
        returnValue.AddStep(mStepFactory.CreateFollowPathAndWarmShooterStep("SimpleTurn.1", 36));
        return returnValue;
    }

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
        listOfPairs.add(new Pair<Double,Long>(30.0, 2500l));
        listOfPairs.add(new Pair<Double,Long>(21.0, 100000l));
    

        returnValue.AddStep(mStepFactory.CreateFollowTrajectoryAndInstaShootStep("4CloseRingAuto.2", listOfPairs));
        returnValue.AddStep(mStepFactory.CreateFinishStep());
        
        return returnValue;

    }

    public AutonomousRoutine FiveCloseRingTopMiddleAuto()
    {
        double initialAngle = 36;
        double instaShootAngle = 25;
        AutonomousRoutine returnValue = new AutonomousRoutine("FiveCloseRingTopMiddleAuto");
        //returnValue.AddStep(mStepFactory.CreateGameStartStep());
        returnValue.AddStep(mStepFactory.CreateFollowPathAndWarmShooterStep("FiveCloseRingTopMiddleAuto.1", initialAngle));
        returnValue.AddStep(mStepFactory.CreateShootStep());

        ArrayList<Pair<Double,Long>> listOfPairs = new ArrayList<>();
        listOfPairs.add(new Pair<Double,Long>(27.0, 1250l));
        listOfPairs.add(new Pair<Double,Long>(30.0, 2500l));
        listOfPairs.add(new Pair<Double,Long>(21.0, 100000l));
    

        returnValue.AddStep(mStepFactory.CreateFollowTrajectoryAndInstaShootStep("FiveCloseRingTopMiddleAuto.2", listOfPairs));

        ArrayList<Pair<Double,Long>> unknownAmpSideAngle = new ArrayList<>();
        unknownAmpSideAngle.add(new Pair<Double,Long>(27.0, 100000l));

        returnValue.AddStep(mStepFactory.CreateFollowPathThenIntakePieceAndHold("FiveCloseRingTopMiddleAuto.3", null, unknownAmpSideAngle));
        returnValue.AddStep(mStepFactory.CreateShootStep());
        
        returnValue.AddStep(mStepFactory.CreateFinishStep());
        
        return returnValue;

    }


    public AutonomousRoutine FiveCloseRingTopMiddleAutoShifted()
    {
        double initialAngle = 36;
        double instaShootAngle = 25;
        AutonomousRoutine returnValue = new AutonomousRoutine("FiveCloseRingTopMiddleAutoShifted");
        //returnValue.AddStep(mStepFactory.CreateGameStartStep());
        returnValue.AddStep(mStepFactory.CreateFollowPathAndWarmShooterStep("FiveCloseRingTopMiddleAutoShifted.1", initialAngle));
        returnValue.AddStep(mStepFactory.CreateShootStep());

        ArrayList<Pair<Double,Long>> listOfPairs = new ArrayList<>();
        listOfPairs.add(new Pair<Double,Long>(27.0, 1250l));
        listOfPairs.add(new Pair<Double,Long>(30.0, 2500l));
        listOfPairs.add(new Pair<Double,Long>(21.0, 100000l));
    

        returnValue.AddStep(mStepFactory.CreateFollowTrajectoryAndInstaShootStep("FiveCloseRingTopMiddleAutoShifted.2", listOfPairs));

        ArrayList<Pair<Double,Long>> unknownAmpSideAngle = new ArrayList<>();
        unknownAmpSideAngle.add(new Pair<Double,Long>(6.0, 100000l));

        returnValue.AddStep(mStepFactory.CreateFollowPathThenIntakePieceAndHold(
            "FiveCloseRingTopMiddleAutoShifted.3", null, unknownAmpSideAngle));
        returnValue.AddStep(mStepFactory.CreateWaitStep(300l));
        returnValue.AddStep(mStepFactory.CreateShootStep());
        
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
    

    public AutonomousRoutine ShootCloseToStageCloseToSourceSide()
    {
         double initialAngle = 34;
        AutonomousRoutine returnValue = new AutonomousRoutine("ShootCloseToStageCloseToSourceSide");
        //returnValue.AddStep(mStepFactory.CreateGameStartStep());
        returnValue.AddStep(mStepFactory.CreateFollowPathAndWarmShooterStep("ShootCloseToStageCloseToSourceSide.1", initialAngle));
        returnValue.AddStep(mStepFactory.CreateShootStep());

        ArrayList<Pair<Double,Long>> eightTeenDegrees = new ArrayList<>();
        eightTeenDegrees.add(new Pair<Double,Long>(18.0, 100000l));
        returnValue.AddStep(mStepFactory.CreateFollowTrajectoryAndInstaShootStep("ShootCloseToStageCloseToSourceSide.2", eightTeenDegrees));
        
        ArrayList<Pair<Double,Long>> secondAngle = new ArrayList<>();
        secondAngle.add(new Pair<Double,Long>(39.0, 100000l));
        
        returnValue.AddStep(mStepFactory.CreateFollowPathThenIntakePieceAndHold("ShootCloseToStageCloseToSourceSide.3", null, secondAngle));
        returnValue.AddStep(mStepFactory.CreateShootStep());
        ArrayList<Pair<Double,Long>> finalShotAngle = new ArrayList<>(); // completely irrelevant 
        finalShotAngle.add(new Pair<Double,Long>(30.0, 100000l));
        returnValue.AddStep(mStepFactory.CreateFollowPathThenIntakePieceAndHold("ShootCloseToStageCloseToSourceSide.4", null, finalShotAngle));
        returnValue.AddStep(mStepFactory.CreateFinishStep());
        
        return returnValue;
    }

    public AutonomousRoutine AvoidanceShootCloseToStageCloseToSourceSide()
    {
        double initialAngle = 34;
        AutonomousRoutine returnValue = new AutonomousRoutine("AvoidanceShootCloseToStageCloseToSourceSide");
        //returnValue.AddStep(mStepFactory.CreateGameStartStep());
        returnValue.AddStep(mStepFactory.CreateWaitStep(6000l));
        
        returnValue.AddStep(mStepFactory.CreateFollowPathAndWarmShooterStep("AvoidanceShootCloseToStageCloseToSourceSide.1", initialAngle));
        returnValue.AddStep(mStepFactory.CreateShootStep());

        ArrayList<Pair<Double,Long>> eightTeenDegrees = new ArrayList<>();
        eightTeenDegrees.add(new Pair<Double,Long>(18.0, 100000l));
        returnValue.AddStep(mStepFactory.CreateFollowTrajectoryAndInstaShootStep("AvoidanceShootCloseToStageCloseToSourceSide.2", eightTeenDegrees));
        
        ArrayList<Pair<Double,Long>> thirtySixDegrees = new ArrayList<>();
        thirtySixDegrees.add(new Pair<Double,Long>(36.0, 100000l));
        
        returnValue.AddStep(mStepFactory.CreateFollowPathThenIntakePieceAndHold(
            "AvoidanceShootCloseToStageCloseToSourceSide.3", null, thirtySixDegrees));
        returnValue.AddStep(mStepFactory.CreateShootStep());
        returnValue.AddStep(mStepFactory.CreateFollowPathThenIntakePieceAndHold("AvoidanceShootCloseToStageCloseToSourceSide.4", null, eightTeenDegrees));
        returnValue.AddStep(mStepFactory.CreateFinishStep());
        
        return returnValue;
    }

    public AutonomousRoutine SourceSideCrazyModeAvoidStage()
    {
        double initialAngle = 34;
        AutonomousRoutine returnValue = new AutonomousRoutine("SourceSideCrazyModeAvoidStage");
        //to let shooter wheels warm up
        returnValue.AddStep(mStepFactory.CreateWaitStep(500l));
        
        returnValue.AddStep(mStepFactory.CreateFollowPathAndWarmShooterStep("SourceSideCrazyModeAvoidStage.1", initialAngle));
        returnValue.AddStep(mStepFactory.CreateShootStep());

        ArrayList<Pair<Double,Long>> unknownSourceSideAngle = new ArrayList<>();
        unknownSourceSideAngle.add(new Pair<Double,Long>(27.0, 100000l));
        
        returnValue.AddStep(mStepFactory.CreateFollowPathThenIntakePieceAndHold(
            "SourceSideCrazyModeAvoidStage.2", null, unknownSourceSideAngle));
        returnValue.AddStep(mStepFactory.CreateShootStep());
 
        returnValue.AddStep(mStepFactory.CreateFollowPathThenIntakePieceAndHold(
            "SourceSideCrazyModeAvoidStage.3", null, unknownSourceSideAngle));
        returnValue.AddStep(mStepFactory.CreateShootStep());

        ArrayList<Pair<Double,Long>> unknownAmpSideAngle = new ArrayList<>();
        unknownAmpSideAngle.add(new Pair<Double,Long>(27.0, 100000l));
        

        returnValue.AddStep(mStepFactory.CreateFollowPathThenIntakePieceAndHold(
            "SourceSideCrazyModeAvoidStage.4", null, unknownAmpSideAngle));
        returnValue.AddStep(mStepFactory.CreateFinishStep());
        
        return returnValue;
    }

    public AutonomousRoutine SourceSideCrazierModeUnderStage()
    {
        double initialAngle = 14;
        AutonomousRoutine returnValue = new AutonomousRoutine("SourceSideCrazierModeUnderStage");
        //to let shooter wheels warm up
        
        
        returnValue.AddStep(mStepFactory.CreateFollowPathAndWarmShooterStep("SourceSideCrazierModeUnderStage.1", initialAngle));
        returnValue.AddStep(mStepFactory.CreateWaitStep(500l));
        returnValue.AddStep(mStepFactory.CreateShootStep());

        ArrayList<Pair<Double,Long>> unknownUnderStageAngle = new ArrayList<>();
        unknownUnderStageAngle.add(new Pair<Double,Long>(13.0, 100000l));
        
        returnValue.AddStep(mStepFactory.CreateFollowPathThenIntakePieceAndHold(
            "SourceSideCrazierModeUnderStage.2", null, unknownUnderStageAngle));
        returnValue.AddStep(mStepFactory.CreateWaitStep(300l));
        returnValue.AddStep(mStepFactory.CreateShootStep());
 
        returnValue.AddStep(mStepFactory.CreateFollowPathThenIntakePieceAndHold(
            "SourceSideCrazierModeUnderStage.3", null, unknownUnderStageAngle));
        returnValue.AddStep(mStepFactory.CreateShootStep());

        returnValue.AddStep(mStepFactory.CreateFollowPathThenIntakePieceAndHold(
            "SourceSideCrazierModeUnderStage.4", null, unknownUnderStageAngle));
        returnValue.AddStep(mStepFactory.CreateShootStep());
        returnValue.AddStep(mStepFactory.CreateFinishStep());
        
        return returnValue;
    }

    public AutonomousRoutine AmpSideCrazy5Note()
    {
        double initialAngle = 34;
        AutonomousRoutine returnValue = new AutonomousRoutine("AmpSideCrazy5Note");
        //to let shooter wheels warm up
        returnValue.AddStep(mStepFactory.CreateWaitStep(500l));
        
        returnValue.AddStep(mStepFactory.CreateFollowPathAndWarmShooterStep("AmpSideCrazy5Note.1", initialAngle));
        returnValue.AddStep(mStepFactory.CreateShootStep());

        ArrayList<Pair<Double,Long>> instaShootAngle = new ArrayList<>();
        instaShootAngle.add(new Pair<Double,Long>(18.0, 100000l));
        returnValue.AddStep(mStepFactory.CreateFollowTrajectoryAndInstaShootStep("AmpSideCrazy5Note.2", instaShootAngle));
        

        ArrayList<Pair<Double,Long>> unknownAmpSideAngle = new ArrayList<>();
        unknownAmpSideAngle.add(new Pair<Double,Long>(27.0, 100000l));
        
 
        returnValue.AddStep(mStepFactory.CreateFollowPathThenIntakePieceAndHold(
            "AmpSideCrazy5Note.3", null, unknownAmpSideAngle));
        returnValue.AddStep(mStepFactory.CreateShootStep());

        returnValue.AddStep(mStepFactory.CreateFollowPathThenIntakePieceAndHold(
            "AmpSideCrazy5Note.4", null, unknownAmpSideAngle));
        returnValue.AddStep(mStepFactory.CreateShootStep());

        returnValue.AddStep(mStepFactory.CreateFollowPathThenIntakePieceAndHold(
            "AmpSideCrazy5Note.5", null, unknownAmpSideAngle));
        returnValue.AddStep(mStepFactory.CreateShootStep());


        returnValue.AddStep(mStepFactory.CreateFinishStep());
        
        return returnValue;
    }

    public AutonomousRoutine AmpSideCrazyInstaShootRushMiddleNoPreload4Note()
    {
        double initialAngle = 34;
        AutonomousRoutine returnValue = new AutonomousRoutine("AmpSideCrazyInstaShootRushMiddleNoPreload4Note");
        //to let shooter wheels warm up



        ArrayList<Pair<Double,Long>> instaShootAngle = new ArrayList<>();
        instaShootAngle.add(new Pair<Double,Long>(18.0, 100000l));
        returnValue.AddStep(mStepFactory.CreateFollowTrajectoryAndInstaShootStep(
            "AmpSideCrazyInstaShootRushMiddleNoPreload4Note.1", instaShootAngle));
        

        ArrayList<Pair<Double,Long>> unknownAmpSideAngle = new ArrayList<>();
        unknownAmpSideAngle.add(new Pair<Double,Long>(27.0, 100000l));
        
        returnValue.AddStep(mStepFactory.CreateFollowPathThenIntakePieceAndHold(
            "AmpSideCrazyInstaShootRushMiddleNoPreload4Note.2", null, unknownAmpSideAngle));
        returnValue.AddStep(mStepFactory.CreateShootStep());
 
        returnValue.AddStep(mStepFactory.CreateFollowPathThenIntakePieceAndHold(
            "AmpSideCrazyInstaShootRushMiddleNoPreload4Note.3", null, unknownAmpSideAngle));
        returnValue.AddStep(mStepFactory.CreateShootStep());

        returnValue.AddStep(mStepFactory.CreateFollowPathThenIntakePieceAndHold(
            "AmpSideCrazyInstaShootRushMiddleNoPreload4Note.4", null, unknownAmpSideAngle));
        returnValue.AddStep(mStepFactory.CreateShootStep());

        //get final note but dont shoot

        returnValue.AddStep(mStepFactory.CreateFollowPathThenIntakePieceAndHold(
            "AmpSideCrazyInstaShootRushMiddleNoPreload4Note.5", null, unknownAmpSideAngle));
        
        returnValue.AddStep(mStepFactory.CreateFinishStep());
        
        return returnValue;
    }

    public AutonomousRoutine AmpSideAllMiddleRushCraziest4Note()
    {
        double initialAngle = 34;
        AutonomousRoutine returnValue = new AutonomousRoutine("AmpSideAllMiddleRushCraziest4Note");
        //to let shooter wheels warm up


        ArrayList<Pair<Double,Long>> unknownAmpSideAngle = new ArrayList<>();
        unknownAmpSideAngle.add(new Pair<Double,Long>(27.0, 100000l));
        
        returnValue.AddStep(mStepFactory.CreateFollowPathThenIntakePieceAndHold(
            "AmpSideAllMiddleRushCraziest4Note.1", null, unknownAmpSideAngle));
        returnValue.AddStep(mStepFactory.CreateShootStep());
 
        returnValue.AddStep(mStepFactory.CreateFollowPathThenIntakePieceAndHold(
            "AmpSideAllMiddleRushCraziest4Note.2", null, unknownAmpSideAngle));
        returnValue.AddStep(mStepFactory.CreateShootStep());

        ArrayList<Pair<Double,Long>> unknownUnderStageAngle = new ArrayList<>();
        unknownUnderStageAngle.add(new Pair<Double,Long>(27.0, 100000l));

        returnValue.AddStep(mStepFactory.CreateFollowPathThenIntakePieceAndHold(
            "AmpSideAllMiddleRushCraziest4Note.3", null, unknownUnderStageAngle));
        returnValue.AddStep(mStepFactory.CreateShootStep());

        //get final note but dont shoot

        returnValue.AddStep(mStepFactory.CreateFollowPathThenIntakePieceAndHold(
            "AmpSideAllMiddleRushCraziest4Note.4", null, unknownUnderStageAngle));
        returnValue.AddStep(mStepFactory.CreateShootStep());

        //get try to get out of the stage but probably run out of time

        returnValue.AddStep(mStepFactory.CreateFollowPathThenIntakePieceAndHold(
            "AmpSideAllMiddleRushCraziest4Note.5", null, unknownUnderStageAngle));
        returnValue.AddStep(mStepFactory.CreateFinishStep());
        
        return returnValue;
    }

    

    public AutonomousRoutine PlayoffsA()
    {
        double initialAngle = 14;
        AutonomousRoutine returnValue = new AutonomousRoutine("PlayoffsA");
        //returnValue.AddStep(mStepFactory.CreateGameStartStep());
        returnValue.AddStep(mStepFactory.CreateFollowPathAndWarmShooterStep("PlayoffsA.1", initialAngle));
        returnValue.AddStep(mStepFactory.CreateWaitStep(1000l));
        returnValue.AddStep(mStepFactory.CreateShootStep());

        // ArrayList<Pair<Double,Long>> eightTeenDegrees = new ArrayList<>();
        // eightTeenDegrees.add(new Pair<Double,Long>(18.0, 100000l)); // NEEDS ADJUSTMENT
        // returnValue.AddStep(mStepFactory.CreateFollowTrajectoryAndInstaShootStep("PlayoffsA.2", eightTeenDegrees));
        
        ArrayList<Pair<Double,Long>> thirtySixDegrees = new ArrayList<>(); // NEEDS ADJUSTMENT
        thirtySixDegrees.add(new Pair<Double,Long>(14.0, 100000l));
        
        returnValue.AddStep(mStepFactory.CreateFollowPathThenIntakePieceAndHold("PlayoffsA.2", null, thirtySixDegrees));
        returnValue.AddStep(mStepFactory.CreateShootStep());
        returnValue.AddStep(mStepFactory.CreateFollowPathThenIntakePieceAndHold("PlayoffsA.3", null, thirtySixDegrees));
        returnValue.AddStep(mStepFactory.CreateShootStep());
        returnValue.AddStep(mStepFactory.CreateFinishStep());
        
        return returnValue;
    }
}
