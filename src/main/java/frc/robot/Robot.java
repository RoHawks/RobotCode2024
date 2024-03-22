// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Subsystem;
import robosystems.ClimberArms;
import robosystems.ExtendoArm;
import robosystems.Intake;
import robosystems.Lights;
import robosystems.Shooter;
import robotcode.autonomous.AutonomousRoutine;

import states.StateMachine;
import states.JoystickControlsWithSwerve;
import states.ShooterMode;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.DriverStation.Alliance;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import universalSwerve.SwerveDrive;
import universalSwerve.SwerveFactory;
import universalSwerve.components.WheelLabel;

import java.util.ArrayList;

import com.choreo.lib.Choreo;
import com.choreo.lib.ChoreoTrajectory;
import com.choreo.lib.ChoreoTrajectoryState;


/**
 * The VM is configured to automatically run this class, and to call the functions corresponding to
 * each mode, as described in the TimedRobot documentation. If you change the name of this class or
 * the package after creating this project, you must also update the build.gradle file in the
 * project.
 */
public class Robot extends TimedRobot 
{
  private class AutonomousEntry
  {
    private int mIndex;
    private String mName;
    private AutonomousRoutine mRoutine;
    
    public AutonomousEntry(int pIndex, String pName, AutonomousRoutine pRoutine)
    {
      mIndex = pIndex;
      mName = pName;
      mRoutine = pRoutine;
    }

    public int GetIndex()
    {
      return mIndex;
    }

    public String GetName()
    {
      return mName;
    }

    public AutonomousRoutine GetRoutine()
    {
      return mRoutine;
    }
  }


  //MAIN COMPONENTS
  private SwerveDrive mSwerveDrive;
  private ExtendoArm mExtendoArm;
  private Intake mIntake;
  private Shooter mShooter;
  private Lights mLights;
  private LimelightManager mLimelightManager;
  private ClimberArms mClimberArms;
  private boolean mAnglerMotorAndGyroZeroingHasOccurred;
  private StateMachine mStateMachine;
  private XboxController mMainController;  
  private XboxController mAlternateController;
  private JoystickControlsWithSwerve mControls;

  private ArrayList<AutonomousEntry> mAutonomousRoutines = new ArrayList<AutonomousEntry>();

  

  public Robot()
  {
    super(0.005);//ovverride the loop timing... it was 20ms, set it to 10ms.
  }

  private void CreateMainComponents()
  {
    mLimelightManager = LimelightManager.GetInstance();
    mLights = new Lights();
    mIntake = new Intake();
    mShooter = new Shooter(mLimelightManager);    
    mExtendoArm = new ExtendoArm();
    mClimberArms = new ClimberArms();
    mSwerveDrive = SwerveFactory.Create2024Swerve();
   
  }

  private void CreateControls()
  {
    mMainController = new XboxController(0);
    mAlternateController = new XboxController(1);    
    mControls = new JoystickControlsWithSwerve(mMainController, mAlternateController);
  }

  
  private AutonomousRoutine mAutonomousRoutine;
  /**
   * This function is run when the robot is first started up and should be used for any
   * initialization code.
   */

  @Override
  public void robotInit() {

    mAnglerMotorAndGyroZeroingHasOccurred = false;
    CreateMainComponents();
    CreateControls();
    
    mStateMachine = new StateMachine(mControls, mSwerveDrive, mIntake, mShooter, mExtendoArm, mClimberArms, mLights, mLimelightManager);    
    
    
    initializeAutonomousRoutines();

   

  }


  private void initializeAutonomousRoutines()
  {
    int currentIndex = 0;
    robotcode.autonomous.RoutineFactory routineFactory = new robotcode.autonomous.RoutineFactory(mSwerveDrive, mShooter, mIntake);
    mAutonomousRoutines.add(new AutonomousEntry(currentIndex++, "Four Close Rings", routineFactory.FourCloseRingAuto()));
    mAutonomousRoutines.add(new AutonomousEntry(currentIndex++, "Amp Side Two Notes Plus One Midfield", routineFactory.OnLeftTwoNotePlusMidfield()));
    mAutonomousRoutines.add(new AutonomousEntry(currentIndex++, "Shoot Close To Stage Close To Source Side", routineFactory.ShootCloseToStageCloseToSourceSide()));
    mAutonomousRoutines.add(new AutonomousEntry(currentIndex++, "Avoidance Shoot Close To Stage Close To Source Side", routineFactory.AvoidanceShootCloseToStageCloseToSourceSide()));
    
  }


  private boolean mLoggingEnabled = true;
  private boolean mLastLoggingEnabledTogglePressed = false;

  private void checkLoggingEnabledToggle()
  {
    boolean changeButtonIsPressed = mControls.GetLoggingEnabledToggle();
    
    if(changeButtonIsPressed && !mLastLoggingEnabledTogglePressed)
    {
      mLoggingEnabled = !mLoggingEnabled;
    }

    mLastLoggingEnabledTogglePressed = changeButtonIsPressed;
  }

  private void logMainMechanisms()
  {
    SmartDashboard.putBoolean("Forward Limit", mShooter.testGetForwardAnglerLimitSwitchTriggered());
    SmartDashboard.putBoolean("Reverse Limit", mShooter.testGetReverseAnglerLimitSwitchTriggered());


    if(mLoggingEnabled)
    {
      //SmartDashboard.putNumber("Logging At", System.currentTimeMillis());
      mLimelightManager.logLimelightInfo();
      
      mShooter.logShooterInformation();
      mStateMachine.log();
      //mExtendoArm.logExtendoArm();
      
      //mClimberArms.logArms();
      
      //mIntake.logIntake();
      //mShooter.logShooterInformation();
  
      /*
      mSwerveDrive.EnableDiagnostics();
      mSwerveDrive.EnableDiagnostics(WheelLabel.NE);
      mSwerveDrive.EnableDiagnostics(WheelLabel.SE);
      mSwerveDrive.EnableDiagnostics(WheelLabel.SW);
      mSwerveDrive.EnableDiagnostics(WheelLabel.NW);
      mSwerveDrive.LogDiagnostics();
      */
    }
    else
    {
      mSwerveDrive.DisableDiagnostics();
      mSwerveDrive.DisableDiagnostics(WheelLabel.NE);
      mSwerveDrive.DisableDiagnostics(WheelLabel.SE);
      mSwerveDrive.DisableDiagnostics(WheelLabel.SW);
      mSwerveDrive.DisableDiagnostics(WheelLabel.NW);
    }
  }

  @Override
  public void robotPeriodic()
  {    
  
    try
    {
      SmartDashboard.putString("Raw alliance Data", DriverStation.getAlliance().get().toString());
    }
    catch(Exception e)
    {
      SmartDashboard.putString("Raw alliance Data", e.getMessage());

    }

    mLimelightManager.calculateCameraPoseTargetSpace();
    mLimelightManager.calculateBotpose();    

    checkLoggingEnabledToggle();
    logMainMechanisms();
  }

  

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
  private long mAutonomousStartTime = 0;
  @Override
  public void autonomousInit() 
  {
    AllianceInfo.GetInstance().LoadAllianceFromFMS();
    //AllianceInfo.GetInstance().OverrideAllianceForTestingPurposes(Alliance.Red);

    mAutonomousRoutine = mAutonomousRoutines.get(mAutoModeSelection).GetRoutine();
    mAutonomousStartTime = System.currentTimeMillis();
  
    
  }

  /** This function is called periodically during autonomous. */
  @Override
  public void autonomousPeriodic() {
    if (!mAnglerMotorAndGyroZeroingHasOccurred)
    {
      zeroAnglerEncoderAndGyro();
      return;
    }
    try
    {
      mAutonomousRoutine.Run();
    }
    catch (Exception e)
    {
      SmartDashboard.putString("error", e.getLocalizedMessage());
    }
    //SmartDashboard.putNumber("AutonomousElapsedTime",System.currentTimeMillis() -  mAutonomousStartTime );
  }


  private long mTeleopStartTime;

  /** This function is called once when teleop is enabled. */
  @Override
  public void teleopInit() 
  {
    mTeleopStartTime = System.currentTimeMillis();
    AllianceInfo.GetInstance().LoadAllianceFromFMS();
    try
    {
      SmartDashboard.putString("OurAllianceInfo", AllianceInfo.GetInstance().GetName());
    }
    catch(Exception e)
    {
      SmartDashboard.putString("OurAllianceInfo", e.getMessage());
    }

    //AllianceInfo.GetInstance().OverrideAllianceForTestingPurposes(Alliance.Red);

    mShooter.SetAnglerLimitSwitchesEnabledOrDisabled(true);
  }
  

  public void zeroAnglerEncoderAndGyro_IfLimitSwitchesAreBroken()
  {
    //until we fix limit switch....

            mSwerveDrive.SetGyroscopeCurrentAngle(0); // remember to set this back to 0 for real game.  180 is for testing where you can ssee the foal.
    
        mAnglerMotorAndGyroZeroingHasOccurred = true;
        mShooter.setAnglerSpeed(0);
        mShooter.zeroAnglerMotorInformation();
        Constants.setLowGoalRotation();
        mSwerveDrive.InitializeOdometry();

  }

  public void zeroAnglerEncoderAndGyro()
  {
    //SmartDashboard.putBoolean("Limit Reached", mShooter.mAnglerHardReverseLimitSwitchisPressed());        
    if (!mAnglerMotorAndGyroZeroingHasOccurred)
    {
      mShooter.setAnglerSpeed(-0.15);
      
      if (mShooter.isAnglerHardReverseLimitSwitchPressed())
      {
        mSwerveDrive.SetGyroscopeCurrentAngle(0); // remember to set this back to 0 for real game.  180 is for testing where you can ssee the foal.
    
        mAnglerMotorAndGyroZeroingHasOccurred = true;
        mShooter.setAnglerSpeed(0);
        mShooter.zeroAnglerMotorInformation();
        Constants.setLowGoalRotation();
        mSwerveDrive.InitializeOdometry();
      }
    }   
  }

  public void testAnglerLimits(){

  }

  boolean mStarted = false;
  boolean mFirstHit = false;
  boolean mPersSpeed = false;
  double started = 0;
  private void testTimedShooterTest()
  {
    mShooter.spinUpToHighGoalSpeed();
    if (!mStarted)
    {
      mStarted = true;
      started = System.currentTimeMillis();
      SmartDashboard.putNumber("time when started", System.currentTimeMillis());
    }

    if (!mFirstHit && Math.abs(mShooter.getTopSpeed() - Constants.SHOOTER_HIGH_TOP_DEFAULT_SPEED) < 1)
    {
      mFirstHit = true;
      SmartDashboard.putNumber("time when first reached speed", System.currentTimeMillis());
      SmartDashboard.putNumber("delta time when first reached speed", System.currentTimeMillis() - started);
    }

    if (!mPersSpeed && mShooter.checkIfPersistentlyHasCorrectSpeed(ShooterMode.HighGoalDriveBy))
    {
      mPersSpeed = true;
      SmartDashboard.putNumber("time when persistently correct speed", System.currentTimeMillis());
      SmartDashboard.putNumber("delta time when persistently correct speed", System.currentTimeMillis() - started);
    }
  }

  private void shootySpinnyTrapTest(){
    
    double angleChange = (mMainController.getLeftTriggerAxis() - mMainController.getRightTriggerAxis()) / 10.0;
    mTestOnlyAnlgerTracker += angleChange;
    mShooter.setAngle(mTestOnlyAnlgerTracker);

    if (mMainController.getYButton())
    {
      mClimberArms.extend();
    }
    else if (mMainController.getXButton())
    {
      mClimberArms.retract();
    }
    

    
    // int pov = mMainController.getPOV(); 
    
    // if(Math.abs(pov - 0.0) < 0.1)
    // {
    //   mExtendoTargetPosition -= 1.0;
    // } 
    // else if(Math.abs(pov - 180.0) < 0.1)
    // {
    //   mExtendoTargetPosition += 1.0;
    // }

    // mExtendoArm.goToPosition(mExtendoTargetPosition);


    
    double topWheels = mMainController.getRightY(); 
    if( Math.abs(topWheels) > 0.07)
    {
      mTestOnlyShooterSpeedTop+= -1.0 * topWheels / 100;
    }
    
    double bottomWheels = mMainController.getLeftY();
    if( Math.abs(bottomWheels) > 0.07)
    {
      mTestOnlyShooterSpeedBottom += -1.0 * bottomWheels / 100;
    }
    SmartDashboard.putNumber("mTestOnlyShooterSpeedBottom", mTestOnlyShooterSpeedBottom);
    SmartDashboard.putNumber("mTestOnlyShooterSpeedTop", mTestOnlyShooterSpeedTop);
    mShooter.setSpeed(mTestOnlyShooterSpeedTop, mTestOnlyShooterSpeedBottom); 

    
    
    if(mMainController.getStartButton())
    {
      mIntake.setToLaunchingNoteIntoTheShooterSpeed();
    }
    else
    {
      mIntake.stopMotors();
    }

  }

  
  private double recordedTurningAngle = 0;
  private boolean triggedPreviousTime = false;
  private boolean hasSetAnything = false;
  private long timeWhenCloseEnough = -1;
  int i = 0;
  public void autoAngleShootTest()
  {
    
    
    SmartDashboard.putNumber("AutoLog: timeWhenCloseEnough", timeWhenCloseEnough);
    SmartDashboard.putNumber("AutoLog: recordedTurningAngle", recordedTurningAngle);
    if (!triggedPreviousTime && mControls.GetStartShootingSequence())
    {
      hasSetAnything = true;
      recordedTurningAngle = LimelightInformation.GetAngleToAprilTag(mLimelightManager.getBotPoseByPriorityCamera(LimelightManager.EAST_CAMERA));
      angleShooterToTheAprilTag();
      i = 0;
      timeWhenCloseEnough = -1;
    }
    
    if (hasSetAnything)
    {
      turnToTheAprilTag(recordedTurningAngle);
      double gyroAngle = mSwerveDrive.GetGyroscopeCurrentAngle(); 
      boolean closeEnough = Math.abs(gyroAngle - recordedTurningAngle) < 7;

      SmartDashboard.putNumber("AutoLog: gyroAngle", gyroAngle);
      SmartDashboard.putBoolean("AutoLog: closeEnough", closeEnough);
      SmartDashboard.putNumber("AutoLog: displacementFromDesiredAngle", Math.abs(gyroAngle - recordedTurningAngle));

      if (closeEnough && timeWhenCloseEnough < 0)
      {
        timeWhenCloseEnough = System.currentTimeMillis();
      }
      else if (!closeEnough)
      {
        timeWhenCloseEnough = -1;
      }

      if (timeWhenCloseEnough > 0 && System.currentTimeMillis() - timeWhenCloseEnough > 1000)
      {
        // mShooter.checkIfPersistentlyHasCorrectSpeed(ShooterMode.HighGoalManual);
        // double angle = LimelightInformation.GetAngleToAprilTag(mLimelightManager.getBotPoseByPriorityCamera(LimelightManager.EAST_CAMERA));
        if (mShooter.checkIfAnglerIsCloseEnough()) // gyroAngle == angle &&
        {
          SmartDashboard.putNumber("AutoLog: is Shooting", System.currentTimeMillis());
          mIntake.setToLaunchingNoteIntoTheShooterSpeed(); 
          mShooter.spinUpToHighGoalSpeed();
        }
        else 
        {
          // recordedTurningAngle = LimelightInformation.GetAngleToAprilTag(mLimelightManager.getBotPoseByPriorityCamera(LimelightManager.EAST_CAMERA));
          // angleShooterToTheAprilTag();
          i++;
          SmartDashboard.putNumber("i", i);

        }
      }
      else
      {
        mShooter.spinUpToHighGoalSpeed();
        mIntake.setToHoldingSpeed();
    }
    }
    triggedPreviousTime = mControls.GetStartShootingSequence();
     
  }


  public void teleopPeriodic()
  {
    SmartDashboard.putNumber("Time Into Teleop", System.currentTimeMillis() - mTeleopStartTime);
    // autoAngleShootTest();
    SmartDashboard.putNumber("AllianceInfoAimOffset", AllianceInfo.GetInstance().GetCentralAprilTagDistanceFromDriversRightWall());
    if (!mAnglerMotorAndGyroZeroingHasOccurred)
    {
      zeroAnglerEncoderAndGyro();
      return;
    }    

    

    mShooter.SetAnglerLimitSwitchesEnabledOrDisabled(true);
    mStateMachine.Run();      
  
  }

  /** This function is called once when the robot is disabled. */
  @Override
  public void disabledInit() {}

  private int mAutoModeSelection = 0;
  private boolean mAutoModeSelectionSwitchButtonLastWasPressed = false;
  private void selectAutoMode()
  {
    boolean changeButtonIsPressed = mControls.GetAutonomousModeSelection();
    if(changeButtonIsPressed && !mAutoModeSelectionSwitchButtonLastWasPressed)
    {
      mAutoModeSelection = ((mAutoModeSelection + 1) % mAutonomousRoutines.size());
    }
    mAutoModeSelectionSwitchButtonLastWasPressed = changeButtonIsPressed;

    SmartDashboard.putString("AUTONOMOUS_MODE", mAutonomousRoutines.get(mAutoModeSelection).GetName());
    
  }

  /** This function is called periodically when disabled. */
  @Override
  public void disabledPeriodic() 
  {
    selectAutoMode();
  }

  /** This function is called once when test mode is enabled. */
  @Override
  public void testInit() 
  {
    mShooter.SetAnglerLimitSwitchesEnabledOrDisabled(false);
    mSwerveDrive.InitializeOdometry();

  }


  private void testExtendoArmPID()
  {
    if(mMainController.getBButton()) 
    {
      mExtendoArm.lowGoalExtension();
    }
    else if(mMainController.getXButton()) 
    {
      mExtendoArm.retract();
    }
    else
    {
      mExtendoArm.stopMotor();
    }
  }


  private double mTestOnlyAnlgerTracker = 0;
  private boolean mTestOnlyHasSeenTrapRing = false;
  private double mTestOnlyShooterSpeed = 0;

  public void testTrapMechanism()
  {

//    mSwerveDrive.Run(mControls);
      
    
    double angleChange = (mMainController.getLeftTriggerAxis() - mMainController.getRightTriggerAxis()) / 10.0;
    mTestOnlyAnlgerTracker += angleChange;
    mShooter.setAngle(mTestOnlyAnlgerTracker);

    double armSpeed = 0.5;
    if (mMainController.getYButton())
    {
      mClimberArms.extend();
    }
    else if (mMainController.getXButton())
    {
      mClimberArms.retract();
    }
    else
    {
      mClimberArms.TEST_ONLY_SetRightArmSpeed(0);      
      mClimberArms.TEST_ONLY_SetLeftArmSpeed(0);      
    }


    
    int pov = mMainController.getPOV(); 
    
    if(Math.abs(pov - 0.0) < 0.1)
    {
      mExtendoTargetPosition -= 1.0;
    } 
    else if(Math.abs(pov - 180.0) < 0.1)
    {
      mExtendoTargetPosition += 1.0;
    }

    mExtendoArm.goToPosition(mExtendoTargetPosition);


    if(!mTestOnlyHasSeenTrapRing)
    {
      mTestOnlyHasSeenTrapRing = mExtendoArm.GetTrapLimitSwitch();
    }
    if(mMainController.getRightBumper() && !mTestOnlyHasSeenTrapRing)
    {
      mIntake.TestSetTrapIntakeSpeed(-1.0);
    }
    else
    {
       mIntake.TestSetTrapIntakeSpeed(0);
    }

     
    if( Math.abs(mMainController.getLeftY()) > 0.07)
    {
      mTestOnlyShooterSpeed += -1.0 * mMainController.getLeftY() / 100;
    }
    mShooter.setSpeed(mTestOnlyShooterSpeed, mTestOnlyShooterSpeed * 0.75);
    
    
    if(mMainController.getRightStickButton())
    {
      mIntake.setToLaunchingNoteIntoTheShooterSpeed();
    }
    else
    {
      mIntake.stopMotors();
    }

    SmartDashboard.putNumber("TestMode-ExtendoArmEncoder", mExtendoArm.getEncoderReading());
    SmartDashboard.putNumber("TestMode-ExtendoArmTarget", mExtendoTargetPosition);

    SmartDashboard.putNumber("TestMode-AnglerAngle", mShooter.GetAnglerEncoderReading());
    SmartDashboard.putNumber("TestMode-AnglerTargetAngle", mTestOnlyAnlgerTracker);

    SmartDashboard.putBoolean("TestMode-TrapLimitSwitchA", mExtendoArm.GetTrapLimitSwitch());
    SmartDashboard.putNumber("TestMode-mTestOnlyShooterSpeed", mTestOnlyShooterSpeed);
  }

  private double Ternary(boolean pConditionA, double pValueA, boolean pConditionB, double pValueB, double pValueElse)
  {
    return pConditionA ? pValueA : (pConditionB ? pValueB : pValueElse);
  }

  private void RealTestMode()
  {
      mShooter.TestOnly_SetAnglerSpeed(Ternary(mControls.TestOnly_AnglerUp(), 0.2, mControls.TestOnly_AnglerDown(), -0.2, 0));
      mShooter.TestOnly_SetTopShooterSpeed(mControls.TestOnly_TopShooterOn() ? 0.75 : 0);
      mShooter.TestOnly_SetBottomShooterSpeed(mControls.TestOnly_BottomShooterOn() ? 0.75 : 0);

      mIntake.setSpeeds(
        mControls.TestOnly_TopConveyorOn() ? 0.75 : 0,
        mControls.TestOnly_BottomConveyorOn() ? 0.75 : 0,
        mControls.TestOnly_IntakeRollerOn() ? 0.75 : 0
      );

      mClimberArms.TEST_ONLY_SetLeftArmSpeed(Ternary(mControls.TestOnly_WestArmUp(), 0.25 * 0.25, mControls.TestOnly_WestArmDown(), -0.25 * 0.25, 0));
      mClimberArms.TEST_ONLY_SetRightArmSpeed(Ternary(mControls.TestOnly_EastArmUp(), -0.25 * 0.25, mControls.TestOnly_EastArmDown(), 0.25 * 0.25, 0));

      mExtendoArm.testOnlyRunAtSpeed(Ternary(mControls.TestOnly_ExtendoArmOut(), -0.25, mControls.TestOnly_ExtendoArmIn(), 0.25, 0));

      mIntake.TestSetTrapIntakeSpeed(mControls.TestOnly_TrapMechanismOn() ? 0.25 : 0 );

      if(mControls.TestOnly_AllowSwerveDuringTestMode())
      {        
        mSwerveDrive.Run(mControls);
      }
      else
      {        
        mSwerveDrive.StopEverything();
      }
      
      SmartDashboard.putBoolean("Break Beam Status", mIntake.getBreakBeamStatus());
  }

  private void TestClimberPID()
  {
    if(mMainController.getXButton())
    {
      mClimberArms.extend();
    }
    else if(mMainController.getYButton())
    {
      mClimberArms.retract();
    }
    else
    {
      mClimberArms.TEST_ONLY_SetLeftArmSpeed(0);
      mClimberArms.TEST_ONLY_SetRightArmSpeed(0);
    }
  }

  public void testPeriodic()
  {    
    //testTrapMechanism();
    //SwerveDrive.Run(mControls);
    //testExtendoArmPID();
    //TestJustSwerve();
    //TestClimberPID();
    // testSpinnyTrapShot();
    //shootySpinnyTrapTest();
    RealTestMode();
    
  }

  private double mTestOnlyShooterSpeedTop = 0;
  private double mTestOnlyShooterSpeedBottom = 0;
  public void testSpinnyTrapShot()
  {
    
    double topWheels = mMainController.getRightY(); 
    if( topWheels > 0.07)
    {
      mTestOnlyShooterSpeedTop+= -1.0 * topWheels / 100;
    }
    
    double bottomWheels = mMainController.getLeftY();
    if( Math.abs(bottomWheels) > 0.07)
    {
      mTestOnlyShooterSpeedBottom += -1.0 * bottomWheels / 100;
    }
    SmartDashboard.putNumber("mTestOnlyShooterSpeedTop", mTestOnlyShooterSpeedBottom);
    SmartDashboard.putNumber("mTestOnlyShooterSpeedTop", mTestOnlyShooterSpeedTop);
    mShooter.setSpeed(mTestOnlyShooterSpeedTop, mTestOnlyShooterSpeedBottom);

    if(mMainController.getStartButton())
    {
      mIntake.setToLaunchingNoteIntoTheShooterSpeed();
    }
    else
    {
      mIntake.stopMotors();
    }



  }

  public void turnAndAngleToTheAprilTag(double pTurningAngle)
  {
    turnToTheAprilTag(pTurningAngle);
    angleShooterToTheAprilTag();
  }

  public void turnToTheAprilTag(double pAngle)
  {
    mSwerveDrive.Run(mControls, true, pAngle);
  }

  public void angleShooterToTheAprilTag()
  {
    mShooter.setAngleBasedOnShooterMode(ShooterMode.AutoAim);
  }


  private double mTestLowGoalBottomShooterSpeed = 26.4;
  private double mTestLowGoalTopShooterSpeed = 7.8;
  private double mExtendoTargetPosition = 0;
  private void testLowGoal()
  {
    double speedDelta = 0.1;
    if(mMainController.getAButton())
    {
      mTestLowGoalBottomShooterSpeed -= speedDelta;
    }
    else if(mMainController.getBButton())
    {
      mTestLowGoalBottomShooterSpeed += speedDelta;
    }


    if(mMainController.getXButton())
    {
      mTestLowGoalTopShooterSpeed -= speedDelta;
    }
    else if(mMainController.getYButton())
    {
      mTestLowGoalTopShooterSpeed += speedDelta;
    }

    mShooter.setSpeed(mTestLowGoalTopShooterSpeed, mTestLowGoalBottomShooterSpeed);
    mShooter.setAnglerSpeed((mMainController.getLeftTriggerAxis() - mMainController.getRightTriggerAxis()) / 10.0);

    SmartDashboard.putNumber("mTestLowGoalTopShooterSpeed", mTestLowGoalTopShooterSpeed);
    SmartDashboard.putNumber("mTestLowGoalBottomShooterSpeed", mTestLowGoalBottomShooterSpeed);
    SmartDashboard.putNumber("anglerAngle", mShooter.GetAnglerEncoderReading());

    if(mMainController.getRightBumper())
    {
      mIntake.setToLaunchingNoteIntoTheShooterSpeed();
    }
    else
    {
      //mIntake.setToHoldingSpeed();
      mIntake.stopMotors();
    }

    int pov = mMainController.getPOV(); 
    SmartDashboard.putNumber("POV", pov);
    
    if(Math.abs(pov - 0.0) < 0.1)
    {
      mExtendoTargetPosition -= 1.0;
    } 
    else if(Math.abs(pov - 180.0) < 0.1)
    {
      mExtendoTargetPosition += 1.0;
    }

    mExtendoArm.goToPosition(mExtendoTargetPosition);
    SmartDashboard.putNumber("ExtendoTarget", mExtendoTargetPosition);
    SmartDashboard.putNumber("ExtendoCurrent", mExtendoArm.getCurrentDraw());
    SmartDashboard.putNumber("ExtendoEncoder", mExtendoArm.getEncoderReading());

  }

  /** This function is called once when the robot is first started up. */
  @Override
  public void simulationInit() {}

  /** This function is called periodically whilst in simulation. */
  @Override
  public void simulationPeriodic() {}
}
