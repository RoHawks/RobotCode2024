package frc.robot.routines;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.SparkMaxPIDController;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.Encoder;
import frc.robot.arm.LowerJointConstants;
import frc.robot.arm.UpperJointConstants;
import states.Controls;

public class StepFactory 
{

    //private DoubleSolenoid mLeftFingerSolenoid;
    private DoubleSolenoid mRightFingerSolenoid;
    private DoubleSolenoid mArmClamp;
    private CANSparkMax mWristMotor;
    private Encoder mWristEncoder;
    
    private SparkMaxPIDController mLowerJointPIDController;
    private SparkMaxPIDController mUpperJointPIDController;
    private CANSparkMax mLowerJointSparkMax;
    private CANSparkMax mUpperJointSparkMax;


    private TrapezoidProfile.Constraints mLowerJointConstraints;
    private TrapezoidProfile.Constraints mUpperJointConstraints;
    private TrapezoidProfile.Constraints mUpperJointConstraintsForBigDownMovements;
    private TrapezoidProfile.Constraints mLowerJointConstraintsForMovementsCloseToGround;
    private TrapezoidProfile.Constraints mUpperJointConstraintsForMovementsCloseToGround;
    private TrapezoidProfile.Constraints mLowerJointConstraintsForMovementsCloseToGround_SuperSlowLikeAtHofstra;
    private TrapezoidProfile.Constraints mUpperJointConstraintsForMovementsCloseToGround_SuperSlowLikeAtHofstra;

    private AbsoluteEncoder mLowerJointEncoder;
    private AbsoluteEncoder mUpperJointEncoder;
    
    private Controls mControls;
    
    public ClawStepFactory ClawStepsFactory;
    public TimeDelayStepFactory TimeDelayStepsFactory;
    public ClampStepFactory ClampStepsFactory;
    public LowerJointStepFactory LowerJointStepsFactory;
    public UpperJointStepFactory UpperJointStepsFactory;
    public TwistStepFactory TwistStepsFactory;
    public UserInputStepFactory UserInputStepsFactory;

    public StepFactory(
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
        TrapezoidProfile.Constraints pUpperJointConstraintsForMovementsCloseToGround_SuperSlowLikeAtHofstra)
    {
        //mLeftFingerSolenoid = pLeftFingerSolenoid;
        mRightFingerSolenoid = pRightFingerSolenoid;
        mArmClamp = pArmClamp;
        mWristMotor = pWristMotor;
        mWristEncoder = pWristEncoder;
        mLowerJointPIDController = pLowerJointPIDController;
        mUpperJointPIDController = pUpperJointPIDController;      
        mLowerJointConstraints = pLowerJointConstraints;
        mUpperJointConstraints = pUpperJointConstraints;
        mUpperJointConstraintsForBigDownMovements = pUpperJointConstraintsForBigDownMovements;
        mUpperJointEncoder = pUpperJointEncoder;
        mLowerJointEncoder = pLowerJointEncoder;
        mLowerJointConstraintsForMovementsCloseToGround = pLowerJointConstraintsForMovementsCloseToGround;
        mUpperJointConstraintsForMovementsCloseToGround = pUpperJointConstraintsForMovementsCloseToGround;
        mLowerJointSparkMax = pLowerJointSparkMax;
        mUpperJointSparkMax = pUpperointSparkMax;
        mControls = pControls;
        mLowerJointConstraintsForMovementsCloseToGround_SuperSlowLikeAtHofstra = pLowerJointConstraintsForMovementsCloseToGround_SuperSlowLikeAtHofstra;
        mUpperJointConstraintsForMovementsCloseToGround_SuperSlowLikeAtHofstra = pUpperJointConstraintsForMovementsCloseToGround_SuperSlowLikeAtHofstra;
        ClawStepsFactory = new ClawStepFactory();
        TimeDelayStepsFactory = new TimeDelayStepFactory();
        ClampStepsFactory = new ClampStepFactory();
        LowerJointStepsFactory = new LowerJointStepFactory();
        UpperJointStepsFactory = new UpperJointStepFactory();
        TwistStepsFactory = new TwistStepFactory();
        UserInputStepsFactory = new UserInputStepFactory();
    }

    public class ClawStepFactory
    {
        public ClawStep CreateClawStep(String pName, boolean pShouldClose, long pTimeToComplete)
        {
          return new ClawStep(pName, null, 
          //mLeftFingerSolenoid,
           mRightFingerSolenoid, pShouldClose, pTimeToComplete);
        }
      
        public ClawStep CreateOpenClawStep(String pName, long pTimeToComplete)
        {
          return CreateClawStep(pName, false, pTimeToComplete) ;
        }
      
      
        public ClawStep CreateCloseClawStep(String pName, long pTimeToComplete)
        {
          return CreateClawStep(pName, true, pTimeToComplete) ;
        }
    }

    public class TimeDelayStepFactory
    {  
        public TimeDelayStep CreatePauseBeforeUntwistStep(String pName, boolean pSkipThisStep)
        {
            return new TimeDelayStep(pName, null, 750, pSkipThisStep);
        }

        public TimeDelayStep CreatePauseBeforeUpperArmGoesToScoreStep(String pName)
        {
            return new TimeDelayStep(pName, null, 1000, false); //for now, save some time here.  Should maybe say like 800;
        }

        public TimeDelayStep CreatePauseInsteadOfUserInput(String pName)
        {
            return new TimeDelayStep(pName, null, 1, false);
        }

        public TimeDelayStep CreateParameterizedPause(String pName, long pTimeMs)
        {
            return new TimeDelayStep(pName, null, pTimeMs, false);
        }

        public TimeDelayStep CreateParameterizedPause(String pName, long pTimeMs, boolean pSkipThisStep)
        {
            return new TimeDelayStep(pName, null, pTimeMs, pSkipThisStep);
        }
    }

    public class ClampStepFactory
    {
  
        public ClampStep CreateReleaseClampStep(String pName)
        {
            return CreateClampStep(pName, false);
        }
    
        public ClampStep CreateCloseClampStep(String pName)
        {
            return CreateClampStep(pName, true);
        }
    
        private ClampStep CreateClampStep(String pName, boolean pShouldClose)
        {
            return new  ClampStep(pName, null, mArmClamp, pShouldClose);
        }
    }

    public class UpperJointStepFactory
    {
        public ArmMovementComponentStep CreateUpperDownToGrabLocationStep()
        {
          return new ArmMovementComponentStep("UpperDownToGrabLocationStep", null, mUpperJointConstraintsForMovementsCloseToGround, UpperJointConstants.GRAB_LOCATION, mUpperJointPIDController, mUpperJointEncoder, 0.02, -0.02, mUpperJointSparkMax, false);
        }
        
        public ArmMovementComponentStep CreateUpperDownToGrabLocationStepForCube()
        {
          return new ArmMovementComponentStep("UpperDownToGrabLocationStep", null, mUpperJointConstraintsForMovementsCloseToGround, UpperJointConstants.GRAB_LOCATION_FOR_CUBE, mUpperJointPIDController, mUpperJointEncoder, 0.02, -0.02, mUpperJointSparkMax, false);
        }

        public ArmMovementComponentStep CreateUpperDownToGrabLocationStepForCubeAuto()
        {
            return new ArmMovementComponentStep("UpperDownToGrabLocationStep", null, mUpperJointConstraintsForMovementsCloseToGround, UpperJointConstants.GRAB_LOCATION_FOR_CUBE_FOR_AUTO_ONLY, mUpperJointPIDController, mUpperJointEncoder, 0.02, -0.02, mUpperJointSparkMax, false);
        }
                
        public ArmMovementComponentStep CreateUpperDownToGrabLocationStepForCone()
        {
            return new ArmMovementComponentStep("UpperDownToGrabLocationStep", null, mUpperJointConstraintsForMovementsCloseToGround_SuperSlowLikeAtHofstra, 0.124, mUpperJointPIDController, mUpperJointEncoder, 0.02, -0.02, mUpperJointSparkMax, false);
        }
        
        public ArmMovementComponentStep CreateUpperJointMidConeStep()
        {
            return new ArmMovementComponentStep("UpperJointToMidCone", null, mUpperJointConstraints, UpperJointConstants.MID_CONE, mUpperJointPIDController, mUpperJointEncoder, 0.01, -0.01, mUpperJointSparkMax, false);
        }

        public ArmMovementComponentStep CreateUpperToRotationSpotStep(boolean pSkipThisStep)
        {
            return new ArmMovementComponentStep("UpperToRotationSpotStep", null, mUpperJointConstraintsForMovementsCloseToGround, UpperJointConstants.ROTATION_SPOT, mUpperJointPIDController, mUpperJointEncoder, 0.01, -0.01, mUpperJointSparkMax, pSkipThisStep);
        }

        public ArmMovementComponentStep CreateUpperIntakingStep()
        {
            return new ArmMovementComponentStep("upperJointBackToIntake", null, mUpperJointConstraintsForBigDownMovements, UpperJointConstants.INTAKING, mUpperJointPIDController, mUpperJointEncoder, 0.01, -0.01, mUpperJointSparkMax, false);
        }

        public ArmMovementComponentStep CreateUpperJointTopConeStep()
        {
            return new ArmMovementComponentStep("UpperJointToMidCone", null, mUpperJointConstraints, UpperJointConstants.TOP_CONE, mUpperJointPIDController, mUpperJointEncoder, 0.01, -0.01, mUpperJointSparkMax, false);
        }

        public ArmMovementComponentStep CreateUpperScoreParameterized(double pAngle)
        {
            return new ArmMovementComponentStep("UpperScoreParameterized", null, mUpperJointConstraints, pAngle, mUpperJointPIDController, mUpperJointEncoder, 0.01, -0.01, mUpperJointSparkMax, false);
            
        }
        
    }

    public class LowerJointStepFactory
    {

        public ArmMovementComponentStep CreateLowerJointRetractionStep()
        {
          ArmMovementComponentStep returnValue = new ArmMovementComponentStep("LowerJointRetration", null,  mLowerJointConstraints, LowerJointConstants.SCOOCHED_BACKWARDS, mLowerJointPIDController, mLowerJointEncoder, 0.01, -0.01, mLowerJointSparkMax, false);
          returnValue.SetInitialVelocity(-0.05);
          return returnValue;
        }

        public ArmMovementComponentStep CreateLowerDownToGrabLocationStepForCone()
        {
            return new ArmMovementComponentStep("LowerDownToGrabLocationStep", null,  mLowerJointConstraintsForMovementsCloseToGround_SuperSlowLikeAtHofstra, LowerJointConstants.GRAB_LOCATION, mLowerJointPIDController, mLowerJointEncoder, 0.02, -0.02, mLowerJointSparkMax, false);
        } 

        public ArmMovementComponentStep CreateLowerDownToGrabLocationStep()
        {
            return new ArmMovementComponentStep("LowerDownToGrabLocationStep", null,  mLowerJointConstraintsForMovementsCloseToGround, LowerJointConstants.GRAB_LOCATION, mLowerJointPIDController, mLowerJointEncoder, 0.02, -0.02, mLowerJointSparkMax, false);
        } 

        public ArmMovementComponentStep CreateLowerJointBackwardsToScoreStep(String pName)
        {
            return new ArmMovementComponentStep(pName, null,  mLowerJointConstraintsForMovementsCloseToGround, LowerJointConstants.BACKWARDS_TO_ALLOW_UPPER_TO_UP, mLowerJointPIDController, mLowerJointEncoder, 0.01, -0.01, mLowerJointSparkMax, false);  
        }

  
        public ArmMovementComponentStep CreateLowerJointScoochBackwardsStep(String pName, boolean pSkipThisStep)
        {
          return new ArmMovementComponentStep(pName, null,  mLowerJointConstraintsForMovementsCloseToGround, LowerJointConstants.SCOOCHED_BACKWARDS, mLowerJointPIDController, mLowerJointEncoder, 0.01, -0.01, mLowerJointSparkMax, pSkipThisStep);
        } 
      
        public ArmMovementComponentStep CreateLowerJointMidConeStep()
        {
          return new ArmMovementComponentStep("lowerJointMidCone", null,  mLowerJointConstraints, LowerJointConstants.MID_CONE, mLowerJointPIDController, mLowerJointEncoder, 0.01, -0.01, mLowerJointSparkMax, false);
        } 
      
        public ArmMovementComponentStep CreateLowerIntakeStep()
        {
          return new ArmMovementComponentStep("lowerJointIntake", null,  mLowerJointConstraintsForMovementsCloseToGround, LowerJointConstants.INTAKING, mLowerJointPIDController, mLowerJointEncoder, 0.01, -0.01, mLowerJointSparkMax, false);
        } 
      
        public ArmMovementComponentStep CreateLowerJointTopConeStep()
        {
            return new ArmMovementComponentStep("lowerJointMidCone", null,  mLowerJointConstraints, LowerJointConstants.TOP_CONE, mLowerJointPIDController, mLowerJointEncoder, 0.01, -0.01, mLowerJointSparkMax, false);
        }

        public ArmMovementComponentStep CreateLowerScoreParameterized(double pAngle)
        {
            return new ArmMovementComponentStep("LowerScoreParameterized", null,  mLowerJointConstraints, pAngle, mLowerJointPIDController, mLowerJointEncoder, 0.01, -0.01, mLowerJointSparkMax, false);
        }

    }


    public class TwistStepFactory
    {
        public TwistStep CreateTwistToGrab(boolean pSkipThisStep)
        {
            return new TwistStep("TwistToGrab", null, mWristMotor, mWristEncoder, 3, -3, 0.75, false, pSkipThisStep);
        }

        public TwistStep CreateTwistToScore(boolean pSkipThisStep)
        {
            return new TwistStep("TwistToGrab", null, mWristMotor, mWristEncoder, 3, -3, 0.375, true, pSkipThisStep);
        }
    }

    public class UserInputStepFactory
    {
        public UserInputStep CreateUserInputStep()
        {
            return new UserInputStep("PressRightButton", null, mControls);
        }
    }
}
