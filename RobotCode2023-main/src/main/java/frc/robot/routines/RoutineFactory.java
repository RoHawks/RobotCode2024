package frc.robot.routines;

import java.util.Arrays;
import java.util.List;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.arm.LowerJointConstants;
import frc.robot.arm.UpperJointConstants;
import states.Controls;

public class RoutineFactory 
{
    private StepFactory mStepFactory;

    public RoutineFactory( 
        //DoubleSolenoid pLeftFingerSolenoid,
        DoubleSolenoid pRightFingerSolenoid,
        DoubleSolenoid pArmClamp,
        CANSparkMax pWristMotor,
        Encoder pWristEncoder,        
        SparkMaxPIDController pLowerJointPIDController,
        SparkMaxPIDController pUpperJointPIDController,      
        TrapezoidProfile.Constraints pLowerJointConstraints,
        TrapezoidProfile.Constraints pUpperJointConstraints,
        TrapezoidProfile.Constraints pUpperJointConstraintsForBigDownMovements,
        AbsoluteEncoder pUpperJointEncoder,
        AbsoluteEncoder pLowerJointEncoder,
        TrapezoidProfile.Constraints pLowerJointConstraintsForMovementsCloseToGround,
        TrapezoidProfile.Constraints pUpperJointConstraintsForMovementsCloseToGround,
        Controls pControls,
        CANSparkMax pLowerJointSparkMax,
        CANSparkMax pUpperointSparkMax,
        TrapezoidProfile.Constraints pLowerJointConstraintsForMovementsCloseToGround_SuperSlowLikeAtHofstra,
        TrapezoidProfile.Constraints pUpperJointConstraintsForMovementsCloseToGround_SuperSlowLikeAtHofstra
        )
    {
        mStepFactory = new StepFactory(
            //pLeftFingerSolenoid,
            pRightFingerSolenoid,
            pArmClamp,
            pWristMotor,
            pWristEncoder,        
            pLowerJointPIDController,
            pUpperJointPIDController,      
            pLowerJointConstraints,
            pUpperJointConstraints,
            pUpperJointConstraintsForBigDownMovements,
            pUpperJointEncoder,
            pLowerJointEncoder,
            pLowerJointConstraintsForMovementsCloseToGround,
            pUpperJointConstraintsForMovementsCloseToGround,
            pControls,
            pLowerJointSparkMax,
            pUpperointSparkMax,
            pLowerJointConstraintsForMovementsCloseToGround_SuperSlowLikeAtHofstra,
            pUpperJointConstraintsForMovementsCloseToGround_SuperSlowLikeAtHofstra
        );

    }

 
    public Routine CreateSmoothRoutineParameterized(boolean pRequiresUserInput, double pLowerUpToScore, double pUpperUpToScore)
    {
      
        ClampStep releaseClamp = mStepFactory.ClampStepsFactory.CreateReleaseClampStep("InitalClampRelease");
        ClampStep closeClamp = mStepFactory.ClampStepsFactory.CreateCloseClampStep("FinalClampRelease");

        TimeDelayStep pauseBeforeUpperArmGoesToScore = mStepFactory.TimeDelayStepsFactory.CreatePauseBeforeUpperArmGoesToScoreStep("pauseBeforeUpperArmGoesToScore");
        TimeDelayStep pauseBeforeUntwist = mStepFactory.TimeDelayStepsFactory.CreatePauseBeforeUntwistStep("PauseBeforeUntwist", false);
        
        TimeDelayStep pauseBeforeLowerArmGoesToScore = mStepFactory.TimeDelayStepsFactory.CreateParameterizedPause("PauseBeforeLowerUpToScore", 10);

        TimeDelayStep pauseBeforeUpperGoesBackToIntake = mStepFactory.TimeDelayStepsFactory.CreateParameterizedPause("pauseBeforeUpperGoesBackToIntake", 300);


        TimeDelayStep pauseInsteadOfUserInput = mStepFactory.TimeDelayStepsFactory.CreatePauseInsteadOfUserInput("PauseInsteadOfUserInput");

        ClawStep initialOpen = mStepFactory.ClawStepsFactory.CreateOpenClawStep("InitialOpen", 0);
        ClawStep grab = mStepFactory.ClawStepsFactory.CreateCloseClawStep("Grab", 600);
        ClawStep releaseToScore = mStepFactory.ClawStepsFactory.CreateOpenClawStep("ReleaseToScore", 1000);


        TwistStep twistToGrab = mStepFactory.TwistStepsFactory.CreateTwistToGrab(false);
        TwistStep twistToScore = mStepFactory.TwistStepsFactory.CreateTwistToScore(false);

        ArmMovementComponentStep upperJointUpToRotationSpot = mStepFactory.UpperJointStepsFactory.CreateUpperToRotationSpotStep(false);
        ArmMovementComponentStep upperJointDownToGrabLocationStep = mStepFactory.UpperJointStepsFactory.CreateUpperDownToGrabLocationStep();
        ArmMovementComponentStep upperJointUpToScore = mStepFactory.UpperJointStepsFactory.CreateUpperScoreParameterized(pUpperUpToScore);
        ArmMovementComponentStep upperJointBackToIntake = mStepFactory.UpperJointStepsFactory.CreateUpperIntakingStep();


        ArmMovementComponentStep lowerJointScoochBackwards = mStepFactory.LowerJointStepsFactory.CreateLowerJointScoochBackwardsStep("lowerJointScoochBackwardsToAllowArmUp", false);
        ArmMovementComponentStep lowerJointDownToGrabLocationStep = mStepFactory.LowerJointStepsFactory.CreateLowerDownToGrabLocationStep();
        ArmMovementComponentStep lowerJointScoochBackwards2 = mStepFactory.LowerJointStepsFactory.CreateLowerJointBackwardsToScoreStep("lowerJointScoochBackwardsToAllowArmUp2");    
        ArmMovementComponentStep lowerJointUpToScore = mStepFactory.LowerJointStepsFactory.CreateLowerScoreParameterized(pLowerUpToScore);
        ArmMovementComponentStep lowerJointRetractToAllowUpperToFall = mStepFactory.LowerJointStepsFactory.CreateLowerJointRetractionStep();
        ArmMovementComponentStep lowerJointBackToIntake = mStepFactory.LowerJointStepsFactory.CreateLowerIntakeStep();

        UserInputStep pressToRelease = mStepFactory.UserInputStepsFactory.CreateUserInputStep();
        
        lowerJointScoochBackwards.AddPrerequisite(new StepPrequisite(releaseClamp, 1, -1));

        upperJointUpToRotationSpot.AddPrerequisite(new StepPrequisite(lowerJointScoochBackwards, 0.01, -0.01));

        twistToGrab.AddPrerequisite(new StepPrequisite(upperJointUpToRotationSpot, 0.005, -0.005));

        upperJointDownToGrabLocationStep.AddPrerequisite(new StepPrequisite(twistToGrab, 5, -5));
        lowerJointDownToGrabLocationStep.AddPrerequisite(new StepPrequisite(twistToGrab, 5, -5));
        grab.AddPrerequisite(new StepPrequisite(upperJointDownToGrabLocationStep, 0.005, -0.005));
        grab.AddPrerequisite(new StepPrequisite(lowerJointDownToGrabLocationStep, 0.005, -0.005));
        pauseBeforeUpperArmGoesToScore.AddPrerequisite(new StepPrequisite(grab, 1, -1)); 

        lowerJointScoochBackwards2.AddPrerequisite(new StepPrequisite(grab, 0.01, -0.01));
        pauseBeforeLowerArmGoesToScore.AddPrerequisite(new StepPrequisite(lowerJointScoochBackwards2, 1, -1));
        upperJointUpToScore.AddPrerequisite(new StepPrequisite(pauseBeforeUpperArmGoesToScore, 0.01, -0.01));
        
        

        pauseBeforeUntwist.AddPrerequisite(new StepPrequisite(pauseBeforeUpperArmGoesToScore, 0.01, -0.01));
        twistToScore.AddPrerequisite(new StepPrequisite(pauseBeforeUntwist, 1, -1));    
        
        lowerJointUpToScore.AddPrerequisite(new StepPrequisite(pauseBeforeLowerArmGoesToScore, 1000000, -1));
      
        //lowerJointUpToScore.AddPrerequisite(new StepPrequisite(upperJointUpToScore, 0.06, -0.06));
      
      

        if(pRequiresUserInput)
        {
            pressToRelease.AddPrerequisite(new StepPrequisite(lowerJointScoochBackwards2, 1, -1)); //this is not a strict prereq... just make sure he doesn't hit the button too early.
            releaseToScore.AddPrerequisite(new StepPrequisite(pressToRelease, 1, -1));
        }
        else
        {
            pauseInsteadOfUserInput.AddPrerequisite(new StepPrequisite(upperJointUpToScore, 0.01, -0.01));
            pauseInsteadOfUserInput.AddPrerequisite(new StepPrequisite(lowerJointUpToScore, 0.01, -0.01));
            releaseToScore.AddPrerequisite(new StepPrequisite(pauseInsteadOfUserInput, 1, -1));
    
        }
        releaseToScore.AddPrerequisite(new StepPrequisite(lowerJointUpToScore, 0.01, -0.01));
        releaseToScore.AddPrerequisite(new StepPrequisite(upperJointUpToScore, 0.01, -0.01));

        pauseBeforeUpperGoesBackToIntake.AddPrerequisite(new StepPrequisite(releaseToScore, 0.01, -0.01));


        lowerJointRetractToAllowUpperToFall.AddPrerequisite(new StepPrequisite(releaseToScore, 0.01, -0.01));
        //upperJointBackToIntake.AddPrerequisite(new StepPrequisite(lowerJointRetractToAllowUpperToFall, 0.01, -0.01));
        upperJointBackToIntake.AddPrerequisite(new StepPrequisite(pauseBeforeUpperGoesBackToIntake, 1000000, -1));
        lowerJointBackToIntake.AddPrerequisite(new StepPrequisite(upperJointBackToIntake, 0.01, -0.01));

        //uncomment when ready to test clamp
        closeClamp.AddPrerequisite(new StepPrequisite(upperJointBackToIntake, 0.01, -0.01));
        closeClamp.AddPrerequisite(new StepPrequisite(lowerJointBackToIntake, 0.01, -0.01));

        StepSequence upperJointStepSequence = new StepSequence("UpperArm", Arrays.asList(upperJointUpToRotationSpot, upperJointDownToGrabLocationStep, upperJointUpToScore, upperJointBackToIntake));
        StepSequence lowerJointStepSequence = new StepSequence("LowerArm", Arrays.asList(lowerJointScoochBackwards, lowerJointDownToGrabLocationStep, lowerJointScoochBackwards2, lowerJointUpToScore, lowerJointRetractToAllowUpperToFall, lowerJointBackToIntake));
        StepSequence twistStepSequence = new StepSequence("Wrist", Arrays.asList(twistToGrab, twistToScore));
        StepSequence clawSequence = new StepSequence("Claw", Arrays.asList(initialOpen, grab, releaseToScore));
        StepSequence pauseSequence = new StepSequence("Pauses", Arrays.asList(pauseBeforeUpperArmGoesToScore, pauseBeforeUntwist));
        StepSequence clampSequence  = new StepSequence("Clamp", Arrays.asList(releaseClamp, closeClamp));
        StepSequence userInputSequence = new StepSequence("UserInput", Arrays.asList(pressToRelease));
        StepSequence pausesInsteadOfUserInputs = new StepSequence("PausesInsteadOfUserInput", Arrays.asList(pauseInsteadOfUserInput));
        StepSequence delays2 = new StepSequence("Pauses2", Arrays.asList(pauseBeforeLowerArmGoesToScore));
        StepSequence delays3 = new StepSequence("Pauses3", Arrays.asList(pauseBeforeUpperGoesBackToIntake));
        List<StepSequence> stepSequences;
        if(pRequiresUserInput)
        {
            stepSequences = Arrays.asList(upperJointStepSequence, lowerJointStepSequence, twistStepSequence, clawSequence, pauseSequence, clampSequence, userInputSequence, userInputSequence, delays2, delays3);
        
        }
        else
        {
            stepSequences = Arrays.asList(upperJointStepSequence, lowerJointStepSequence, twistStepSequence, clawSequence, pauseSequence, clampSequence, userInputSequence, pausesInsteadOfUserInputs, delays2, delays3);
        }
        Routine returnValue = new Routine(stepSequences);
        return returnValue;
  
    }
  


    public Routine CreateConeOrCubeRoutineNewStyle(boolean pRequiresUserInput, double pLowerUpToScore, double pUpperUpToScore, boolean pIsCone)
    {
        
        ClampStep releaseClamp = mStepFactory.ClampStepsFactory.CreateReleaseClampStep("InitalClampRelease");
        ClampStep closeClamp = mStepFactory.ClampStepsFactory.CreateCloseClampStep("FinalClampRelease");

        
        TimeDelayStep pauseBeforeLowerArmGoesToScore = mStepFactory.TimeDelayStepsFactory.CreateParameterizedPause("pauseBeforeLowerArmGoesToScore", 125);
        TimeDelayStep pauseBeforeUpperGoesBackToIntake = mStepFactory.TimeDelayStepsFactory.CreateParameterizedPause("pauseBeforeUpperGoesBackToIntake", 300);

        TimeDelayStep pauseBeforeUntwist = mStepFactory.TimeDelayStepsFactory.CreatePauseBeforeUntwistStep("PauseBeforeUntwist", !pIsCone);
      
        TimeDelayStep pauseBeforeLowerArmDownToGrab =  mStepFactory.TimeDelayStepsFactory.CreateParameterizedPause("pauseBeforeLowerArmDownToGrab", 10, !pIsCone);

        ClawStep initialOpen = mStepFactory.ClawStepsFactory.CreateOpenClawStep("InitialOpen", 0);
        ClawStep grab = mStepFactory.ClawStepsFactory.CreateCloseClawStep("Grab", 400);
        ClawStep releaseToScore = mStepFactory.ClawStepsFactory.CreateOpenClawStep("ReleaseToScore", 1000);



        TwistStep twistToGrab = mStepFactory.TwistStepsFactory.CreateTwistToGrab(!pIsCone);
        TwistStep twistToScore = mStepFactory.TwistStepsFactory.CreateTwistToScore(!pIsCone);


        ArmMovementComponentStep upperJointUpToRotationSpot = mStepFactory.UpperJointStepsFactory.CreateUpperToRotationSpotStep(!pIsCone);
        ArmMovementComponentStep upperJointDownToGrabLocationStep;
        if(pIsCone)
        {
            upperJointDownToGrabLocationStep = mStepFactory.UpperJointStepsFactory.CreateUpperDownToGrabLocationStepForCone();
        } 
        else
        {
            if(pRequiresUserInput)
            {
                upperJointDownToGrabLocationStep = mStepFactory.UpperJointStepsFactory.CreateUpperDownToGrabLocationStepForCube();
            }
            else
            {
                upperJointDownToGrabLocationStep = mStepFactory.UpperJointStepsFactory.CreateUpperDownToGrabLocationStepForCubeAuto();
            }
        }
        ArmMovementComponentStep upperJointUpToScore = mStepFactory.UpperJointStepsFactory.CreateUpperScoreParameterized(pUpperUpToScore);
        ArmMovementComponentStep upperJointBackToIntake = mStepFactory.UpperJointStepsFactory.CreateUpperIntakingStep();

        
        ArmMovementComponentStep lowerJointScoochBackwards = mStepFactory.LowerJointStepsFactory.CreateLowerJointScoochBackwardsStep("lowerJointScoochBackwardsToAllowArmUp", !pIsCone);
        ArmMovementComponentStep lowerJointDownToGrabLocationStep = pIsCone ? mStepFactory.LowerJointStepsFactory.CreateLowerDownToGrabLocationStepForCone() : mStepFactory.LowerJointStepsFactory.CreateLowerDownToGrabLocationStep();
        ArmMovementComponentStep lowerJointScoochBackwards2 = mStepFactory.LowerJointStepsFactory.CreateLowerJointBackwardsToScoreStep("lowerJointScoochBackwardsToAllowArmUp2");    
        ArmMovementComponentStep lowerJointUpToScore = mStepFactory.LowerJointStepsFactory.CreateLowerScoreParameterized(pLowerUpToScore);
        ArmMovementComponentStep lowerJointRetractToAllowUpperToFall = mStepFactory.LowerJointStepsFactory.CreateLowerJointRetractionStep();
        ArmMovementComponentStep lowerJointBackToIntake = mStepFactory.LowerJointStepsFactory.CreateLowerIntakeStep();

        UserInputStep pressToRelease = mStepFactory.UserInputStepsFactory.CreateUserInputStep();

        upperJointUpToRotationSpot.AddPrerequisite(new StepPrequisite(lowerJointScoochBackwards, 0.01, -0.01));

        twistToGrab.AddPrerequisite(new StepPrequisite(upperJointUpToRotationSpot, 0.005, -0.005));

        pauseBeforeLowerArmDownToGrab.AddPrerequisite(new StepPrequisite(twistToGrab, 5, -5));

        upperJointDownToGrabLocationStep.AddPrerequisite(new StepPrequisite(twistToGrab, 5, -5));
        //lowerJointDownToGrabLocationStep.AddPrerequisite(new StepPrequisite(twistToGrab, 5, -5));
        lowerJointDownToGrabLocationStep.AddPrerequisite(new StepPrequisite(pauseBeforeLowerArmDownToGrab, 1000000, -1000000));
        grab.AddPrerequisite(new StepPrequisite(upperJointDownToGrabLocationStep, 0.005, -0.005));
        grab.AddPrerequisite(new StepPrequisite(lowerJointDownToGrabLocationStep, 0.005, -0.005));
        
        //pauseBeforeUpperArmGoesToScore.AddPrerequisite(new StepPrequisite(grab, 1, -1)); 

        lowerJointScoochBackwards2.AddPrerequisite(new StepPrequisite(grab, 0.01, -0.01));
        //upperJointUpToScore.AddPrerequisite(new StepPrequisite(pauseBeforeUpperArmGoesToScore, 0.01, -0.01));    

        upperJointUpToScore.AddPrerequisite(new StepPrequisite(lowerJointScoochBackwards2, 0.06, -0.06));    

        pauseBeforeUntwist.AddPrerequisite(new StepPrequisite(lowerJointScoochBackwards2, 0.01, -0.01));
        twistToScore.AddPrerequisite(new StepPrequisite(pauseBeforeUntwist, 1, -1));    

        pauseBeforeLowerArmGoesToScore.AddPrerequisite(new StepPrequisite(lowerJointScoochBackwards2, 1, -1));
        lowerJointUpToScore.AddPrerequisite(new StepPrequisite(pauseBeforeLowerArmGoesToScore, 0.1, -0.1));
      
        pressToRelease.AddPrerequisite(new StepPrequisite(lowerJointScoochBackwards2, 1, -1)); //this is not a strict prereq... just make sure he doesn't hit the button too early.
        if(pRequiresUserInput)
        {
            releaseToScore.AddPrerequisite(new StepPrequisite(pressToRelease, 1, -1));
        }
        releaseToScore.AddPrerequisite(new StepPrequisite(lowerJointUpToScore, 0.01, -0.01));
        releaseToScore.AddPrerequisite(new StepPrequisite(upperJointUpToScore, 0.01, -0.01));
        releaseToScore.AddPrerequisite(new StepPrequisite(twistToScore, 5, -5));

        lowerJointRetractToAllowUpperToFall.AddPrerequisite(new StepPrequisite(releaseToScore, 0.01, -0.01));
        pauseBeforeUpperGoesBackToIntake.AddPrerequisite(new StepPrequisite(releaseToScore, 0.01, -0.01));
        upperJointBackToIntake.AddPrerequisite(new StepPrequisite(pauseBeforeUpperGoesBackToIntake, 1000000, -1));
        //upperJointBackToIntake.AddPrerequisite(new StepPrequisite(lowerJointRetractToAllowUpperToFall, 0.01, -0.01));
        lowerJointBackToIntake.AddPrerequisite(new StepPrequisite(upperJointBackToIntake, 0.09, -0.09));

        closeClamp.AddPrerequisite(new StepPrequisite(upperJointBackToIntake, 0.01, -0.01));
        closeClamp.AddPrerequisite(new StepPrequisite(lowerJointBackToIntake, 0.01, -0.01));

        StepSequence upperJointStepSequence = new StepSequence("UpperArm", Arrays.asList(upperJointUpToRotationSpot, upperJointDownToGrabLocationStep, upperJointUpToScore, upperJointBackToIntake));
        StepSequence lowerJointStepSequence = new StepSequence("LowerArm", Arrays.asList(lowerJointScoochBackwards, lowerJointDownToGrabLocationStep, lowerJointScoochBackwards2, lowerJointUpToScore, lowerJointRetractToAllowUpperToFall, lowerJointBackToIntake));
        StepSequence clawSequence = new StepSequence("Claw", Arrays.asList(initialOpen, grab, releaseToScore));
        //StepSequence pauseSequence = new StepSequence("Pauses", Arrays.asList(pauseBeforeUpperArmGoesToScore));
        StepSequence pauseSequence2 = new StepSequence("Pauses2", Arrays.asList( pauseBeforeLowerArmGoesToScore));
        StepSequence pauseSequence3 = new StepSequence("Pauses3", Arrays.asList( pauseBeforeUpperGoesBackToIntake));
        StepSequence pauseSequence4 = new StepSequence("Pauses4", Arrays.asList( pauseBeforeUntwist));
        StepSequence pauseSequence5 = new StepSequence("Pauses5", Arrays.asList(pauseBeforeLowerArmDownToGrab));
        StepSequence twistSequence = new StepSequence("Wrist", Arrays.asList(twistToGrab, twistToScore));
        
        StepSequence clampSequence  = new StepSequence("Clamp", Arrays.asList(releaseClamp, closeClamp));
        StepSequence userInputSequence = new StepSequence("UserInput", Arrays.asList(pressToRelease));
       
        List<StepSequence> stepSequences;
        if(pRequiresUserInput)
        {
            stepSequences = Arrays.asList(upperJointStepSequence, lowerJointStepSequence, clawSequence, //pauseSequence,
             clampSequence, userInputSequence, pauseSequence2, pauseSequence3, pauseSequence4, twistSequence, pauseSequence5);
        } 
        else
        {
            stepSequences = Arrays.asList(upperJointStepSequence, lowerJointStepSequence, clawSequence, 
            //pauseSequence,
             clampSequence,  pauseSequence2, pauseSequence3, pauseSequence4, twistSequence, pauseSequence5);
        }

        Routine returnValue = new Routine(stepSequences);
        return returnValue;
  
    }

  
    public Routine CreateCubeOnlyRoutineNewStyle(boolean pRequiresUserInput, double pLowerUpToScore, double pUpperUpToScore, boolean pUseSpecialGrabLocation)
    {
        
        ClampStep releaseClamp = mStepFactory.ClampStepsFactory.CreateReleaseClampStep("InitalClampRelease");
        ClampStep closeClamp = mStepFactory.ClampStepsFactory.CreateCloseClampStep("FinalClampRelease");

        //TimeDelayStep pauseBeforeUpperArmGoesToScore = mStepFactory.TimeDelayStepsFactory.CreatePauseBeforeUpperArmGoesToScoreStep("pauseBeforeUpperArmGoesToScore");
        TimeDelayStep pauseBeforeLowerArmGoesToScore = mStepFactory.TimeDelayStepsFactory.CreateParameterizedPause("pauseBeforeLowerArmGoesToScore", 125);
  
        TimeDelayStep pauseBeforeUpperGoesBackToIntake = mStepFactory.TimeDelayStepsFactory.CreateParameterizedPause("pauseBeforeUpperGoesBackToIntake", 300);


        ClawStep initialOpen = mStepFactory.ClawStepsFactory.CreateOpenClawStep("InitialOpen", 0);
        ClawStep grab = mStepFactory.ClawStepsFactory.CreateCloseClawStep("Grab", 400);
        ClawStep releaseToScore = mStepFactory.ClawStepsFactory.CreateOpenClawStep("ReleaseToScore", 1000);

        ArmMovementComponentStep upperJointDownToGrabLocationStep;
        if(pUseSpecialGrabLocation)
        {
            upperJointDownToGrabLocationStep = mStepFactory.UpperJointStepsFactory.CreateUpperDownToGrabLocationStepForCone();
        } 
        else
        {
            upperJointDownToGrabLocationStep = mStepFactory.UpperJointStepsFactory.CreateUpperDownToGrabLocationStepForCube();
        }
        ArmMovementComponentStep upperJointUpToScore = mStepFactory.UpperJointStepsFactory.CreateUpperScoreParameterized(pUpperUpToScore);
        ArmMovementComponentStep upperJointBackToIntake = mStepFactory.UpperJointStepsFactory.CreateUpperIntakingStep();


        ArmMovementComponentStep lowerJointDownToGrabLocationStep = mStepFactory.LowerJointStepsFactory.CreateLowerDownToGrabLocationStep();
        ArmMovementComponentStep lowerJointScoochBackwards2 = mStepFactory.LowerJointStepsFactory.CreateLowerJointBackwardsToScoreStep("lowerJointScoochBackwardsToAllowArmUp2");    
        ArmMovementComponentStep lowerJointUpToScore = mStepFactory.LowerJointStepsFactory.CreateLowerScoreParameterized(pLowerUpToScore);
        ArmMovementComponentStep lowerJointRetractToAllowUpperToFall = mStepFactory.LowerJointStepsFactory.CreateLowerJointRetractionStep();
        ArmMovementComponentStep lowerJointBackToIntake = mStepFactory.LowerJointStepsFactory.CreateLowerIntakeStep();

        UserInputStep pressToRelease = mStepFactory.UserInputStepsFactory.CreateUserInputStep();
        
        upperJointDownToGrabLocationStep.AddPrerequisite(new StepPrequisite(initialOpen, 5, -5));
        lowerJointDownToGrabLocationStep.AddPrerequisite(new StepPrequisite(initialOpen, 5, -5));
        grab.AddPrerequisite(new StepPrequisite(upperJointDownToGrabLocationStep, 0.005, -0.005));
        grab.AddPrerequisite(new StepPrequisite(lowerJointDownToGrabLocationStep, 0.005, -0.005));
        
        //pauseBeforeUpperArmGoesToScore.AddPrerequisite(new StepPrequisite(grab, 1, -1)); 

        lowerJointScoochBackwards2.AddPrerequisite(new StepPrequisite(grab, 0.01, -0.01));
        //upperJointUpToScore.AddPrerequisite(new StepPrequisite(pauseBeforeUpperArmGoesToScore, 0.01, -0.01));    

        upperJointUpToScore.AddPrerequisite(new StepPrequisite(lowerJointScoochBackwards2, 0.06, -0.06));    
        
        pauseBeforeLowerArmGoesToScore.AddPrerequisite(new StepPrequisite(lowerJointScoochBackwards2, 1, -1));
        lowerJointUpToScore.AddPrerequisite(new StepPrequisite(pauseBeforeLowerArmGoesToScore, 0.1, -0.1));
      
        pressToRelease.AddPrerequisite(new StepPrequisite(lowerJointScoochBackwards2, 1, -1)); //this is not a strict prereq... just make sure he doesn't hit the button too early.
        if(pRequiresUserInput)
        {
            releaseToScore.AddPrerequisite(new StepPrequisite(pressToRelease, 1, -1));
        }
        releaseToScore.AddPrerequisite(new StepPrequisite(lowerJointUpToScore, 0.01, -0.01));
        releaseToScore.AddPrerequisite(new StepPrequisite(upperJointUpToScore, 0.01, -0.01));

        lowerJointRetractToAllowUpperToFall.AddPrerequisite(new StepPrequisite(releaseToScore, 0.01, -0.01));
        pauseBeforeUpperGoesBackToIntake.AddPrerequisite(new StepPrequisite(releaseToScore, 0.01, -0.01));
        upperJointBackToIntake.AddPrerequisite(new StepPrequisite(pauseBeforeUpperGoesBackToIntake, 1000000, -1));
        //upperJointBackToIntake.AddPrerequisite(new StepPrequisite(lowerJointRetractToAllowUpperToFall, 0.01, -0.01));
        lowerJointBackToIntake.AddPrerequisite(new StepPrequisite(upperJointBackToIntake, 0.09, -0.09));

        closeClamp.AddPrerequisite(new StepPrequisite(upperJointBackToIntake, 0.01, -0.01));
        closeClamp.AddPrerequisite(new StepPrequisite(lowerJointBackToIntake, 0.01, -0.01));

        StepSequence upperJointStepSequence = new StepSequence("UpperArm", Arrays.asList(upperJointDownToGrabLocationStep, upperJointUpToScore, upperJointBackToIntake));
        StepSequence lowerJointStepSequence = new StepSequence("LowerArm", Arrays.asList(lowerJointDownToGrabLocationStep, lowerJointScoochBackwards2, lowerJointUpToScore, lowerJointRetractToAllowUpperToFall, lowerJointBackToIntake));
        StepSequence clawSequence = new StepSequence("Claw", Arrays.asList(initialOpen, grab, releaseToScore));
        //StepSequence pauseSequence = new StepSequence("Pauses", Arrays.asList(pauseBeforeUpperArmGoesToScore));
        StepSequence pauseSequence2 = new StepSequence("Pauses2", Arrays.asList( pauseBeforeLowerArmGoesToScore));
        StepSequence pauseSequence3 = new StepSequence("Pauses3", Arrays.asList( pauseBeforeUpperGoesBackToIntake));
        
        
        StepSequence clampSequence  = new StepSequence("Clamp", Arrays.asList(releaseClamp, closeClamp));
        StepSequence userInputSequence = new StepSequence("UserInput", Arrays.asList(pressToRelease));
       
        List<StepSequence> stepSequences;
        if(pRequiresUserInput)
        {
            stepSequences = Arrays.asList(upperJointStepSequence, lowerJointStepSequence, clawSequence, //pauseSequence,
             clampSequence, userInputSequence, pauseSequence2, pauseSequence3);
        } 
        else
        {
            stepSequences = Arrays.asList(upperJointStepSequence, lowerJointStepSequence, clawSequence, 
            //pauseSequence,
             clampSequence,  pauseSequence2, pauseSequence3);
        }

        Routine returnValue = new Routine(stepSequences);
        return returnValue;
  
    }

}
