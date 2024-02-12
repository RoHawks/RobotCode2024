// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.routines.Routine;
import frc.robot.routines.RoutineFactory;
import frc.robot.routines.ScoringRoutines;
import robosystems.*;
import states.Controls;
import states.GamePieceLocation;
import states.GamePieces;
import states.JoystickControls;
import states.ScoringInstructions;
import states.StateMachine;
import universalSwerve.SwerveDrive;
import universalSwerve.SwerveFactory;
import universalSwerve.SwerveDrive.SwerveNudgingDirection;
import universalSwerve.utilities.AngleUtilities;
import universalSwerve.utilities.PIDFConfiguration;

import java.util.Set;

import com.revrobotics.AbsoluteEncoder;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.SparkMaxAbsoluteEncoder.Type;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj.PneumaticHub;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.cscore.VideoMode.PixelFormat;
import edu.wpi.first.hal.REVPHVersion;
import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.AddressableLED;
import edu.wpi.first.wpilibj.AddressableLEDBuffer;
import edu.wpi.first.wpilibj.DataLogManager;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DriverStation;



/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot {

  //Declare raw components
  private PneumaticHub mPneumaticHub;

  private CANSparkMax mBottomIntake;
  private CANSparkMax mTopIntake;
  private CANSparkMax mLowerJointSparkMax;
  private CANSparkMax mUpperJointSparkMax;
  private CANSparkMax mWristMotor;

  private AbsoluteEncoder mLowerJointEncoder;
  private AbsoluteEncoder mUpperJointEncoder;
  private Encoder mWristEncoder;

  private DoubleSolenoid mWallSolenoid;
  private DoubleSolenoid mRightFingerSolenoid;
  private DoubleSolenoid mDeploymentSolenoid;  
  private DoubleSolenoid mArmClamp;


  //Declare controllers
  private SparkMaxPIDController mLowerJointPIDController;
  private SparkMaxPIDController mUpperJointPIDController;  
  private PIDController mWristPidController;
  
  private TrapezoidProfile.Constraints mLowerJointConstraints;
  private TrapezoidProfile.Constraints mUpperJointConstraints;
  
  private TrapezoidProfile.Constraints mLowerJointConstraintsForMovementsCloseToGround;
  private TrapezoidProfile.Constraints mUpperJointConstraintsForMovementsCloseToGround;
  private TrapezoidProfile.Constraints mUpperJointConstraintsForMovementsCloseToGround_SuperSlowLikeAtHofstra;
  private TrapezoidProfile.Constraints mLowerJointConstraintsForMovementsCloseToGround_SuperSlowLikeAtHofstra;
  
  private TrapezoidProfile.Constraints mUpperJointConstraintsForBigDownMovements;
  
  private RoutineFactory mRoutineFactory;
  private Routine mAutoRoutine;

  //Declare robot components
  private Intake mIntake;
  private Claw mClaw;
  private Arm mArm;
  private SwerveDrive mSwerveDrive;
  private Wrist mWrist;
  private Wall mWall;

  //Declare misc:
  private XboxController mController;
  private XboxController mAlternateController;
  private Controls mJoystickControls;
  private StateMachine mStateMachine;
  

  //For testing only:
  private double  mLowerJointPIDTestTargetPosition;
  private double mUpperJointPIDTestTargetPosition;  
  private TrapezoidProfile.State mLowerJointSetpoint;
  private TrapezoidProfile.State mUpperJointSetpoint;
  private TrapezoidProfile.State mLowerJointGoal;
  private TrapezoidProfile.State mUpperJointGoal;

  private AddressableLED mLEDs;
  private AddressableLEDBuffer mLEDsBuffer;


  private CANSparkMax CreateWristMotor(int pCANID)
  {
    CANSparkMax returnValue = new CANSparkMax(pCANID, MotorType.kBrushed);
    returnValue.restoreFactoryDefaults();
    returnValue.setSmartCurrentLimit(20);
    returnValue.setIdleMode(IdleMode.kBrake);
    returnValue.burnFlash();
    returnValue.set(0);
    return returnValue;
  }

  private CANSparkMax CreateIntakeMotor(int pCANID)
  {
    CANSparkMax returnValue = new CANSparkMax(pCANID, MotorType.kBrushless);
    returnValue.restoreFactoryDefaults();
    returnValue.setSmartCurrentLimit(20);
    returnValue.burnFlash();
    returnValue.set(0);
    return returnValue;
  }

  private void InitializeMotors()
  {
    mWristMotor = CreateWristMotor(45);
    mBottomIntake = CreateIntakeMotor(3);
    mTopIntake = CreateIntakeMotor(16);    
    /* This is what it was at the end of the season 
    double upperOffsetAfterFridayReconstrction = 0.0864 - 0.02;
    mUpperJointSparkMax = InitializeArmJointSparkMax(13, false, 0.85 +  upperOffsetAfterFridayReconstrction , 0.80, 0.11, new PIDFConfiguration(10, 0, 0, 0), true);
    */
    double upperJointEncoderOffsetForOffseasonTournament = 0.3274; //0.3174;
    mUpperJointSparkMax = InitializeArmJointSparkMax(13, false, upperJointEncoderOffsetForOffseasonTournament, 0.80, 0.11, new PIDFConfiguration(10, 0, 0, 0), true);
    
    mLowerJointSparkMax = InitializeArmJointSparkMax(14, false, 0.28, 0.27, 0.05, new PIDFConfiguration(18, 0, 0, 0), true);
    
  }

  private void InitializeSolenoids()
  {
    //mLeftFingerSolenoid = mPneumaticHub.makeDoubleSolenoid(12,15);
    mWallSolenoid =  mPneumaticHub.makeDoubleSolenoid(12,15);
    mRightFingerSolenoid = mPneumaticHub.makeDoubleSolenoid(11,14);
    mDeploymentSolenoid = mPneumaticHub.makeDoubleSolenoid(10,13);
    mArmClamp = mPneumaticHub.makeDoubleSolenoid(8, 9);
  }

  private void InitializeEncoders()
  {
    mWristEncoder = new Encoder(0, 1);
    mUpperJointEncoder = mUpperJointSparkMax.getAbsoluteEncoder(Type.kDutyCycle);
    mLowerJointEncoder = mLowerJointSparkMax.getAbsoluteEncoder(Type.kDutyCycle);

  }

  private void InitializeComponents()
  {
    mIntake = new Intake(mTopIntake, mBottomIntake, mDeploymentSolenoid, mLEDs, mLEDsBuffer);
    mClaw = new Claw(mRightFingerSolenoid);//, mLeftFingerSolenoid);    
    mArm = new Arm(mUpperJointSparkMax, mLowerJointSparkMax, mArmClamp);
    mWrist = new Wrist(mWristMotor);
    mSwerveDrive = SwerveFactory.Create2023Swerve();
    mSwerveDrive.SetWheelsToCoastMode();
    mWall = new Wall(mWallSolenoid);
  }

  private void InitializeMisc()
  {
    mController = new XboxController(0);
    mAlternateController = new XboxController(1);
    mJoystickControls = new JoystickControls(mController, mAlternateController);
    
    
    mPneumaticHub = new PneumaticHub(31);
    try
    {
      REVPHVersion pcmVersion = mPneumaticHub.getVersion();
      SmartDashboard.putString("PCM INITIALIZATION", "Successful, version " + pcmVersion.toString());
    }
    catch(Exception e)
    {
      SmartDashboard.putString("PCM INITIALIZATION", "FAILED!!!");
    }
    //mPneumaticHub.disableCompressor();
    mPneumaticHub.enableCompressorDigital();
    //UsbCamera camera = CameraServer.startAutomaticCapture();
    //camera.setResolution(320, 240);
    //camera.setFPS(30);
    
    //camera.setPixelFormat(PixelFormat.kGray);
    
    mLEDs = new AddressableLED(0);
    

    mLEDsBuffer = new AddressableLEDBuffer(16);
    mLEDs.setLength(mLEDsBuffer.getLength());
    //mLeftLEDsBuffer = new AddressableLEDBuffer(18);
    //mLeftLEDs.setLength(mLeftLEDsBuffer.getLength());
    for(int i = 0; i < mLEDsBuffer.getLength(); i++)
    {
      mLEDsBuffer.setRGB(i, 0, 0, 0);
    }
    mLEDs.setData(mLEDsBuffer);
    mLEDs.start();
    /*
    for(int i = 0; i < mLeftLEDsBuffer.getLength(); i++)
    {
      mLeftLEDsBuffer.setRGB(i, 0, 255, 0);
    }
    mLeftLEDs.setData(mLeftLEDsBuffer);
    */

  }

  private void InitializeControllers()
  {
    mWristPidController = new PIDController(0.025, 0, 0);
    mUpperJointPIDController = mUpperJointSparkMax.getPIDController();
    mLowerJointPIDController = mLowerJointSparkMax.getPIDController();

    
    mUpperJointSetpoint = new TrapezoidProfile.State(mUpperJointEncoder.getPosition(), 0);
    
    mUpperJointConstraints = new TrapezoidProfile.Constraints(2.5, 0.875);
    mUpperJointConstraintsForMovementsCloseToGround = new TrapezoidProfile.Constraints(0.5, 0.15);
    mUpperJointConstraintsForBigDownMovements =  new TrapezoidProfile.Constraints(0.37, 0.27);
    mUpperJointConstraintsForMovementsCloseToGround_SuperSlowLikeAtHofstra =  new TrapezoidProfile.Constraints(0.5, 0.15);

    mLowerJointConstraints = new TrapezoidProfile.Constraints(1.0, 0.3);
    mLowerJointConstraintsForMovementsCloseToGround = new TrapezoidProfile.Constraints(1.2, 0.25);
    mLowerJointConstraintsForMovementsCloseToGround_SuperSlowLikeAtHofstra = new TrapezoidProfile.Constraints(0.5, 0.09);
        
    mLowerJointSetpoint = new TrapezoidProfile.State(mLowerJointEncoder.getPosition(), 0);    
    mLowerJointPIDTestTargetPosition = mLowerJointEncoder.getPosition();
    mUpperJointPIDTestTargetPosition = mUpperJointEncoder.getPosition();
    mIdealEndSpotUpperJoint = mUpperJointEncoder.getPosition();
    

    

    mRoutineFactory = new RoutineFactory(//mLeftFingerSolenoid,
     mRightFingerSolenoid, mArmClamp, mWristMotor, mWristEncoder, mLowerJointPIDController, mUpperJointPIDController, mLowerJointConstraints, mUpperJointConstraints, mUpperJointConstraintsForBigDownMovements, mUpperJointEncoder, mLowerJointEncoder, mLowerJointConstraintsForMovementsCloseToGround, mUpperJointConstraintsForMovementsCloseToGround, mJoystickControls, mLowerJointSparkMax, mUpperJointSparkMax, mLowerJointConstraintsForMovementsCloseToGround_SuperSlowLikeAtHofstra, mUpperJointConstraintsForMovementsCloseToGround_SuperSlowLikeAtHofstra);    ScoringRoutines.GetInstance().CreateRoutines(mRoutineFactory);
    

  }

  private void InitializeStateMachine()
  {
    mStateMachine = new StateMachine(mJoystickControls, mSwerveDrive, mIntake, mClaw, mWrist, mArm, mWall);
  }

  public void robotInit()
  {
    InitializeMisc();
    InitializeMotors();
    InitializeSolenoids();
    InitializeEncoders();
    InitializeControllers();
    InitializeComponents();
    InitializeStateMachine();

    //DataLogManager.start();
   
  }






  private CANSparkMax InitializeArmJointSparkMax(int pCANID, boolean pIsInverted, double pEncoderOffset, double pForwardLimit, double pReverseLimit, PIDFConfiguration pPIDFConfiguration, boolean pEnableSoftLimit) 
  {
    CANSparkMax sparkMax = new CANSparkMax(pCANID, MotorType.kBrushless);
    sparkMax.restoreFactoryDefaults();
    sparkMax.setSmartCurrentLimit(55);
    sparkMax.setIdleMode(IdleMode.kBrake);
    sparkMax.setInverted(pIsInverted);
    AbsoluteEncoder absoluteEncoder = sparkMax.getAbsoluteEncoder(Type.kDutyCycle);
    absoluteEncoder.setZeroOffset(pEncoderOffset);
    
    SparkMaxPIDController sparkMaxPIDController = sparkMax.getPIDController();
    sparkMaxPIDController.setFeedbackDevice(absoluteEncoder);
    sparkMaxPIDController.setP(pPIDFConfiguration.P());
    sparkMaxPIDController.setI(pPIDFConfiguration.I());
    sparkMaxPIDController.setD(pPIDFConfiguration.D());
    sparkMaxPIDController.setFF(pPIDFConfiguration.F());

    //For now, while trying to figure out the numbers
    sparkMax.enableSoftLimit(CANSparkMax.SoftLimitDirection.kForward, pEnableSoftLimit);
    sparkMax.enableSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, pEnableSoftLimit);

    sparkMax.setSoftLimit(CANSparkMax.SoftLimitDirection.kForward, (float)pForwardLimit);  
    sparkMax.setSoftLimit(CANSparkMax.SoftLimitDirection.kReverse, (float)pReverseLimit);
    
    sparkMax.burnFlash();
    sparkMax.set(0);
    return sparkMax;

  }

  private void LogSparkMax(String pName, CANSparkMax pSparkMax, AbsoluteEncoder pAbsoluteEncoder)
  {
    SmartDashboard.putNumber(pName + "Output", pSparkMax.getAppliedOutput());
    if(pAbsoluteEncoder != null)
    {
      SmartDashboard.putNumber(pName + "Encoder", pAbsoluteEncoder.getPosition());
    }
    SmartDashboard.putNumber(pName + "Current", pSparkMax.getOutputCurrent());
  }


  public double WRIST_TICKS_PER_MOTOR_REVOLUTION = 28.0;
  public double WRIST_GEAR_RATIO = (68.0 / 13.0) * (68.0 / 13.0) * (68.0 / 13.0) ;
  public double ConvertWristEncoderToDegrees(double pEncoderTicks)
  {
    //deg = 360.0 * pEncoderTicks / (WRIST_GEAR_RATIO * WRIST_TICKS_PER_MOTOR_REVOLUTION); 
    return 360.0 * pEncoderTicks / (WRIST_GEAR_RATIO * WRIST_TICKS_PER_MOTOR_REVOLUTION);
  }

  public double ConvertDegreesToWristEncoderTicks(double pDegrees)
  {
    //ticks = (deg / 360) * (WRIST_GEAR_RATIO * WRIST_TICKS_PER_MOTOR_REVOLUTION); 
    return (pDegrees / 360.0) * (WRIST_GEAR_RATIO * WRIST_TICKS_PER_MOTOR_REVOLUTION);
  }
  



  /**
   * This function is called every 20 ms, no matter the mode. Use this for items like diagnostics
   * that you want ran during disabled, autonomous, teleoperated and test.
   *
   * <p>This runs after the mode specific periodic functions, but before LiveWindow and
   * SmartDashboard integrated updating.
   */
  private double mUserOverrideAngle = 0;
  

  @Override
  public void robotPeriodic() 
  {
    LogSparkMax("LowerJoint", mLowerJointSparkMax, mLowerJointEncoder);
    LogSparkMax("UpperJoint", mUpperJointSparkMax, mUpperJointEncoder);
    LogSparkMax("Wrist", mWristMotor, null);
    LogSparkMax("BottomIntake", mBottomIntake, null);
    LogSparkMax("TopIntake", mBottomIntake, null);
    if(mWristEncoder!=null)
    {
      SmartDashboard.putNumber("Encoder Distance: ", mWristEncoder.getDistance());
      SmartDashboard.putNumber("Encoder Raw: ", mWristEncoder.getRaw());
      SmartDashboard.putNumber("WristAngleDegrees:", ConvertWristEncoderToDegrees(mWristEncoder.getRaw()));
    }
    //GetAngleOfUpperArmRelativeToGround();
   
    //SmartDashboard.putNumber("LTrig", mAlternateController.getLeftTriggerAxis());

    SmartDashboard.putString("Goal Mode" ,  mJoystickControls.GetIsLowGoalMode() ? "Low" : "High");
    SmartDashboard.putString("Game Piece" ,  mJoystickControls.GetGamePieceMode() == GamePieces.CONE ? "Cone" : "Cube");
    SmartDashboard.putString("Manual or Normal", mJoystickControls.GetManualControlsMode() ? "Manual" : "Normal");
       
    
  }


  private boolean mClampLastPressed = false;
  private boolean mClampLastValue = false;
  private boolean mClawLastPressed = false;
  private boolean mClawLastValue = true;
  private boolean mIntakeLastPressed = false;
  private boolean mIntakeLastValue = false;
  private double MANUAL_ARM_SPEED = 0.15;
  private boolean mWallLastPressed = false;
  private boolean mWallLastValue = false;
  private void ManualControl()
  {
    boolean clampPressed = mAlternateController.getLeftBumper();
    if(clampPressed && !mClampLastPressed)
    {
      mClampLastValue = !mClampLastValue;
    }
    mClampLastPressed = clampPressed;
    this.mArmClamp.set(mClampLastValue ? Value.kForward : Value.kReverse );

    boolean clawPressed = mAlternateController.getBackButton();
    if(clawPressed && !mClawLastPressed)
    {
      mClawLastValue = !mClawLastValue;      
    }
    mClawLastPressed = clawPressed;
    //this.mLeftFingerSolenoid.set(mClawLastValue ? Value.kForward : Value.kReverse );
    this.mRightFingerSolenoid.set(mClawLastValue ? Value.kForward : Value.kReverse );

    boolean intakePressed = mAlternateController.getLeftStickButton();
    if(intakePressed && !mIntakeLastPressed)
    {
      mIntakeLastValue = !mIntakeLastValue;
    }
    mIntakeLastPressed = intakePressed;
    this.mDeploymentSolenoid.set(mIntakeLastValue ? Value.kForward : Value.kReverse);

    boolean wallPressed = (mAlternateController.getPOV() == 270);//the track button
    if(wallPressed && !mWallLastPressed)
    {
      mWallLastValue = !mWallLastValue;
    }
    mWallLastPressed = wallPressed;
    if(mWallLastValue)
    {
      mWall.Extend();
    }
    else
    {
      mWall.Retract();
    }
    


    if(mAlternateController.getAButton())
    {
      mArmClamp.set(Value.kReverse);
      mUpperJointSparkMax.set(MANUAL_ARM_SPEED);
    }
    else if(mAlternateController.getRightBumper())
    {
      mArmClamp.set(Value.kReverse);
      mUpperJointSparkMax.set(-MANUAL_ARM_SPEED);
    }
    else
    {
      mUpperJointSparkMax.set(0);
    }

    if(mAlternateController.getRightStickButton())
    {
      mArmClamp.set(Value.kReverse);
      mLowerJointSparkMax.set(MANUAL_ARM_SPEED);
    }
    else if(mAlternateController.getStartButton())
    {
      mArmClamp.set(Value.kReverse);
      mLowerJointSparkMax.set(-MANUAL_ARM_SPEED);
    }
    else
    {
      mLowerJointSparkMax.set(0);
    }

    mBottomIntake.set(0);
    mTopIntake.set(0);


    

  }

  private long mLoopTimestamp;
  private boolean mLastModeWasManual = false;
  public void teleopPeriodic()
  {    
    long loopEntry = System.currentTimeMillis();
    SmartDashboard.putNumber("LoopTime", (loopEntry - mLoopTimestamp));
    mLoopTimestamp = loopEntry;
    
    if(mJoystickControls.GetManualControlsMode())
    {
      ManualControl();
      mLastModeWasManual = true;
      
    }
    else
    {
      if(mLastModeWasManual)
      {
        mStateMachine.TransitionBackFromManualMode();
      }


      mStateMachine.Run();
      mLastModeWasManual = false;

      mSwerveDrive.LogDriveData();
    }
   

  }

  private double mAutoBalanceExtra = 0;
  /**
   * This autonomous (along with the chooser code above) shows how to select between different
   * autonomous modes using the dashboard. The sendable chooser code works with the Java
   * SmartDashboard. If you prefer the LabVIEW Dashboard, remove all of the chooser code and
   * uncomment the getString line to get the auto name from the text box below the Gyro
   *
   * <p>You can add additional auto modes by adding additional comparisons to the switch structure
   * below with additional strings. If using the SendableChooser make sure to add them to the
   * chooser code above as well.
   */
  @Override
  public void autonomousInit() {
    InitializeGyroIfHasntBeenAlready();
    mWall.Retract();
    mSwerveDrive.ResetDistanceTravelled();
    mStateMachine.Reset();

    try
    {
      Alliance alliance = DriverStation.getAlliance();
      if(alliance == Alliance.Blue)
      {
        mAutoBalanceExtra = 5;//4;
      }
      else
      {
        mAutoBalanceExtra = 3;//1;
      }
    }
    catch(Exception e)
    {
      System.out.println("Error getting alliance:" + e.getMessage());
    }

    //ATS switched to MID, remember to change back
    mAutoRoutine = ScoringRoutines.GetInstance().GetRoutineForInstructions(
      new ScoringInstructions(GamePieceLocation.HIGH, mJoystickControls.GetGamePieceMode(),
      0, true),
      true);
    
    if(mJoystickControls.GetGamePieceMode() == GamePieces.CONE)
    {
      mAutoRoutine.Reset(80);
    }
  }

  private double AUTO_ANGLE_OFFSET = 0.20;
  private double  GetOffsetForAutoDirection()
  {
    if(mAutoDirectionSelection == AutonomousOptionsForDriveDirection_LEFT)
    {
      return AUTO_ANGLE_OFFSET;
    }
    else if(mAutoDirectionSelection == AutonomousOptionsForDriveDirection_STRAIGHT)
    {
      return 0;
    }
    else if(mAutoDirectionSelection == AutonomousOptionsForDriveDirection_RIGHT)
    {
      return -AUTO_ANGLE_OFFSET;
    }
    else
    {
      return 0;
    }
  }

  /** This function is called periodically during autonomous. */
  private long mAutoStart = 0;
  private double AUTO_TRAVEL_DISTANCE = 185; //seems like it should be 185; //inches?
  // the charge station is 6' 4.5"

  private double AUTO_BALANCE_FULL_SPEED_DISTANCE = 40;
  private double AUTO_BALANCE_DISTANCE = 86; 
  private double AUTO_BALANCE_INITIAL_DRIVE_SPEED = 3.0;
  private double AUTO_BALANCE_FINAL_DRIVE_SPEED = 0.6;
  @Override
  public void autonomousPeriodic()
  {
    if(mAutoStart == 0)//first time
    {
      mAutoStart = System.currentTimeMillis();
    }
    //let's go simple for now:
    mIntake.deploy();
    mWall.Retract();

    if(System.currentTimeMillis() - mAutoStart > 400)
    {
      mAutoRoutine.Run();
    }

    

    if(mAutoModeSelection == AutonomousOptionsForAfterScoring_DRIVE_OUT_OF_COMMUNITY)
    {
      if(mAutoRoutine.IsComplete())
      {
        double distanceTravelled = Math.abs(mSwerveDrive.GetDistanceTravelled());
        SmartDashboard.putNumber("AutoDistance",distanceTravelled);
        if(distanceTravelled < AUTO_TRAVEL_DISTANCE)
        {
          SmartDashboard.putString("AutoNudging", "Yes");
          mSwerveDrive.Nudge(SwerveNudgingDirection.NORTH,3.0, GetOffsetForAutoDirection());
        }
        else
        {
          SmartDashboard.putString("AutoNudging", "No");
          mSwerveDrive.StopEverything();
        } 
      }
      SmartDashboard.putNumber("AutoDistance",mSwerveDrive.GetDistanceTravelled());
    }

    double autoBalanceDistanceWithAllinaceBonus = AUTO_BALANCE_DISTANCE + mAutoBalanceExtra;
    if (mAutoModeSelection == AutonomousOptionsForAfterScoring_GET_ON_BRIDGE)
    {
      if (mAutoRoutine.IsComplete())
      {
        
        
        double distanceTravelled = Math.abs(mSwerveDrive.GetDistanceTravelled());
        SmartDashboard.putNumber("AutoDistance",distanceTravelled);
        if (distanceTravelled < AUTO_BALANCE_FULL_SPEED_DISTANCE)
        {
          double speed = AUTO_BALANCE_INITIAL_DRIVE_SPEED;
          mSwerveDrive.Nudge(SwerveNudgingDirection.NORTH,speed, 0);
          SmartDashboard.putNumber("AutoNudgingSpeed", speed);

        }
        else if(distanceTravelled < autoBalanceDistanceWithAllinaceBonus)
        {
          double percentOfWayBetweenSpeedDropSpotAndFinalSpot = (distanceTravelled - AUTO_BALANCE_FULL_SPEED_DISTANCE) / (autoBalanceDistanceWithAllinaceBonus - AUTO_BALANCE_FULL_SPEED_DISTANCE);
          double speedRange = AUTO_BALANCE_INITIAL_DRIVE_SPEED - AUTO_BALANCE_FINAL_DRIVE_SPEED; 
          double nudgeSpeedMultiplier = AUTO_BALANCE_FINAL_DRIVE_SPEED + (speedRange * (1.0 - percentOfWayBetweenSpeedDropSpotAndFinalSpot));
          mSwerveDrive.Nudge(SwerveNudgingDirection.NORTH, nudgeSpeedMultiplier, 0);
          SmartDashboard.putNumber("AutoNudgingSpeed", nudgeSpeedMultiplier);
        }
        else
        {
          SmartDashboard.putString("AutoNudging", "No");
          mSwerveDrive.TurnAllWheels(90);
          mSwerveDrive.SetWheelsToBreakMode();
          mSwerveDrive.StopTranslationButAllowWheelDirection();
        }
        
      }
    }
  }

  private boolean mHasGyroBeenInitialized = false;
  private void InitializeGyroIfHasntBeenAlready()
  {
    if(!mHasGyroBeenInitialized)
    {
      mSwerveDrive.SetGyroscopeCurrentAngle(180); //we will always start with the intaking facing us.
      mHasGyroBeenInitialized = true;      
    }
  }

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() {
    InitializeGyroIfHasntBeenAlready();
    mSwerveDrive.SetWheelsToCoastMode();
    mStateMachine.Reset();

    //mLeftFingerSolenoid.set(Value.kForward);
    mRightFingerSolenoid.set(Value.kForward);
    mDeploymentSolenoid.set(Value.kForward);
    mArmClamp.set(Value.kForward);
    
   


  } 
    

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  /** This function is called periodically when disabled. */
  private int AutonomousOptionsForAfterScoring_DO_NOTHING = 0;
  private int AutonomousOptionsForAfterScoring_DRIVE_OUT_OF_COMMUNITY = 1;
  private int AutonomousOptionsForAfterScoring_GET_ON_BRIDGE = 2;
  private int AutonomousOptionsForAfterScoring_MAX = AutonomousOptionsForAfterScoring_GET_ON_BRIDGE;
  //private int AutonomousOptionsForAfterScoring_MAX = AutonomousOptionsForAfterScoring_GET_ON_BRIDGE;
  //private int AutonomousOptionsForAfterScoring_DRIVE_OUT_OF_COMMUNITY_THEN_BALANCE_BRIDGE = 3;

  
  private int AutonomousOptionsForDriveDirection_LEFT = 0;
  private int AutonomousOptionsForDriveDirection_STRAIGHT = 1;
  private int AutonomousOptionsForDriveDirection_RIGHT = 2;
  private int AutonomousOptionsForDriveDirection_MAX = AutonomousOptionsForDriveDirection_RIGHT;
  

  private String GetAutoDirectionName()
  {
    if(mAutoDirectionSelection == AutonomousOptionsForDriveDirection_LEFT)
    {
      return "LEFT";
    }
    else if(mAutoDirectionSelection == AutonomousOptionsForDriveDirection_STRAIGHT)
    {
      return "STRAIGHT";
    }
    else if(mAutoDirectionSelection == AutonomousOptionsForDriveDirection_RIGHT)
    {
      return "RIGHT";
    }
    else
    {
      return "Something has gone very wrong... mAutoModeSelection is " + mAutoModeSelection;
    }
  }

  
  private String GetAutoModeName()
  {
    if(mAutoModeSelection == 0)
    {
      return "DO_NOTHING";
    }
    else if(mAutoModeSelection == 1)
    {
      return "DRIVE_OUT_OF_COMMUNITY";
    }
    else if(mAutoModeSelection == 2)
    {
      return "GET_ON_BRIDGE";
    }
    /*
    else if(mAutoModeSelection == 3)
    {
      return "DRIVE_OUT_OF_COMMUNITY_THEN_BALANCE_BRIDGE";
    }
    */
    else
    {
      return "Something has gone very wrong... mAutoModeSelection is " + mAutoModeSelection;
    }
  }


  private boolean mLastAutoModeSwitchButtonPressed = false;
  private int mAutoModeSelection = AutonomousOptionsForAfterScoring_DRIVE_OUT_OF_COMMUNITY;
  
  private boolean mLastAutoDirectionSwitchButtonPressed = false;
  private int mAutoDirectionSelection = AutonomousOptionsForDriveDirection_LEFT;
  
  @Override
  public void disabledPeriodic() 
  {
    boolean modeSwitchAutoButtonPressed = mJoystickControls.GetSwitchAutoOption();
    if(modeSwitchAutoButtonPressed && !mLastAutoModeSwitchButtonPressed)
    {
      mAutoModeSelection = ((mAutoModeSelection + 1) % (AutonomousOptionsForDriveDirection_MAX + 1));
    }
    mLastAutoModeSwitchButtonPressed = modeSwitchAutoButtonPressed;
    SmartDashboard.putString("AUTO_MODE", GetAutoModeName());
  
  
    boolean autoDirectionSwitchButtonPressed = mJoystickControls.GetSwitchAutoDirection();
    if(autoDirectionSwitchButtonPressed && !mLastAutoDirectionSwitchButtonPressed)
    {
      mAutoDirectionSelection = ((mAutoDirectionSelection + 1) % (AutonomousOptionsForAfterScoring_MAX+1));
    }
    mLastAutoDirectionSwitchButtonPressed = autoDirectionSwitchButtonPressed;
    SmartDashboard.putString("AUTO_DIRECTION", GetAutoDirectionName());

  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() {}

  /** This function is called periodically during test mode. */
  @Override
  public void testPeriodic() {}

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}


  
  private double LOWER_ARM_ENCODER_READING_WHILE_VERTICAL = 0.122;
  private double UPPER_ARM_ENCODER_READING_WHILE_LOWER_IS_VERTICAL_AND_UPPER_IS_HORIZONTAL = 0.433;
  private double MAGIC_RATIO = 1.9;
  private double UPPER_SPROCKET_RATIO = 28.0/18.0;


  private double GetAngleOfUpperArmRelativeToGround()
  {

    double currentLowerEncoderReading = mLowerJointEncoder.getPosition();
    double currentUpperEncoderReading = mUpperJointEncoder.getPosition();
    double lowerArmEncoderDifferenceFromVertical = currentLowerEncoderReading - LOWER_ARM_ENCODER_READING_WHILE_VERTICAL;
    SmartDashboard.putNumber("lowerArmEncoderDifferenceFromVertical", lowerArmEncoderDifferenceFromVertical * 360.0);
    double expectedEncoderReadingIfUpperWasHorizontal = UPPER_ARM_ENCODER_READING_WHILE_LOWER_IS_VERTICAL_AND_UPPER_IS_HORIZONTAL
    + (MAGIC_RATIO * lowerArmEncoderDifferenceFromVertical);
    SmartDashboard.putNumber("expectedEncoderReadingIfUpperWasHorizontal", expectedEncoderReadingIfUpperWasHorizontal);
    double upperDifferenceBetweenActualAndHorizontal = currentUpperEncoderReading - expectedEncoderReadingIfUpperWasHorizontal;
    SmartDashboard.putNumber("upperDifferenceBetweenActualAndHorizontal", upperDifferenceBetweenActualAndHorizontal);
    double degreesDifferenceBetweenActualAndHorizontal = (upperDifferenceBetweenActualAndHorizontal * 360.0) / UPPER_SPROCKET_RATIO;
    SmartDashboard.putNumber("degreesDifferenceBetweenActualAndHorizontal", degreesDifferenceBetweenActualAndHorizontal);
    return degreesDifferenceBetweenActualAndHorizontal;

  }


  private PIDController mRoboRioUpperArmPIDController = new PIDController(10, 0,0);
  private ArmFeedforward mUpperFeedforward = new ArmFeedforward(0, 0.03, 0);

  private void TestArmBasicPID()
  {
    double joystickAmount = mController.getRightTriggerAxis() - mController.getLeftTriggerAxis();

    SmartDashboard.putNumber("mLowerJointPIDTestTargetPosition", mLowerJointPIDTestTargetPosition);
    SmartDashboard.putNumber("mUpperJointPIDTestTargetPosition", mUpperJointPIDTestTargetPosition);

    if(mController.getRightBumper())
    {      
      //every 20 ms, move by at most 1/200th of a rotation
      //every second 25/100th of a rotation
      mLowerJointPIDTestTargetPosition += joystickAmount / 200.0;
      mLowerJointPIDController.setReference(mLowerJointPIDTestTargetPosition, ControlType.kPosition);
    }
    else
    {
      mLowerJointSparkMax.set(0);
    }


    if(mController.getLeftBumper())
    {      
      //every 20 ms, move by at most 1/200th of a rotation
      //every second 25/100th of a rotation

      SmartDashboard.putNumber("UpperJointAngleToGround", this.GetAngleOfUpperArmRelativeToGround());

      mUpperJointPIDTestTargetPosition += joystickAmount / 200.0;
      mUpperJointPIDController.setReference(mUpperJointPIDTestTargetPosition, ControlType.kPosition);
      mRoboRioUpperArmPIDController.setSetpoint(mUpperJointPIDTestTargetPosition);
      //double roboRioPIDAndFF = mRoboRioUpperArmPIDController.calculate(mUpperJointEncoder.getPosition()) + mUpperFeedforward.calculate(Math.toRadians(this.GetAngleOfUpperArmRelativeToGround()), 0);
      //mUpperJointSparkMax.set(roboRioPIDAndFF);
      SmartDashboard.putNumber("UpperArmSetPoint", mUpperJointPIDTestTargetPosition);
      SmartDashboard.putNumber("RoboRioPIDOutput", mRoboRioUpperArmPIDController.calculate(mUpperJointEncoder.getPosition()));
      SmartDashboard.putNumber("UpperFeedForward", mUpperFeedforward.calculate(Math.toRadians(this.GetAngleOfUpperArmRelativeToGround()), 0));

    }
    else
    {
      mUpperJointSparkMax.set(0);
    }

  }

  private void UpperJointSmartMotion()
  {
    double kDt = 0.02;

    mUpperJointGoal = new TrapezoidProfile.State(mIdealEndSpotUpperJoint, 0);

    // Create a motion profile with the given maximum velocity and maximum
    // acceleration constraints for the next setpoint, the desired goal, and the
    // current setpoint.
    var profile = new TrapezoidProfile(mUpperJointConstraints, mUpperJointGoal, mUpperJointSetpoint);

    // Retrieve the profiled setpoint for the next timestep. This setpoint moves
    // toward the goal while obeying the constraints.
    mUpperJointSetpoint = profile.calculate(kDt);
    SmartDashboard.putNumber("UpperJointSetPoint", mUpperJointSetpoint.position);
    SmartDashboard.putNumber("UpperJointSetVelocity", mUpperJointSetpoint.velocity);

    // Send setpoint to offboard controller PID
    //mUpperJointPIDController.setReference(mUpperJointSetpoint.position,ControlType.kPosition);
    
    double output = mRoboRioUpperArmPIDController.calculate(mUpperJointEncoder.getPosition()) + mUpperFeedforward.calculate(Math.toRadians(this.GetAngleOfUpperArmRelativeToGround()), 0);
    
    mUpperJointSparkMax.set(output);
  }

  private double UPPER_JOINT_SCORE_LOCATION = 0.5;
  private double UPPER_JOINT_INTAKE_LOCATION = 0.2;
  private double mIdealEndSpotUpperJoint;
  private void UpperSmartMotionTest()
  {
    if(mController.getLeftBumper())
    {
      SmartDashboard.putNumber("mIdealEndSpotUpperJoint", mIdealEndSpotUpperJoint);
      
      if(mController.getXButton())
      {
        mIdealEndSpotUpperJoint = UPPER_JOINT_SCORE_LOCATION;
        UpperJointSmartMotion();
      }    
      else if(mController.getAButton())
      {
        mIdealEndSpotUpperJoint = UPPER_JOINT_INTAKE_LOCATION;
        UpperJointSmartMotion();
      }

      
    }
    else
    {
      mUpperJointSparkMax.set(0);
    }
  }


}
