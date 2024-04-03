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
import robosystems.RobotMode;
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
    
    
    mAutonomousRoutines.add(new AutonomousEntry(currentIndex++, "Five Close Rings ", routineFactory.FiveCloseRingTopMiddleAuto()));
    
    mAutonomousRoutines.add(new AutonomousEntry(currentIndex++, "Four Close Rings", routineFactory.FourCloseRingAuto()));

    mAutonomousRoutines.add(new AutonomousEntry(currentIndex++, "Amp Side Two Notes Plus One Midfield", routineFactory.OnLeftTwoNotePlusMidfield()));
    
    mAutonomousRoutines.add(new AutonomousEntry(currentIndex++, "Shoot Close To Stage Close To Source Side", routineFactory.ShootCloseToStageCloseToSourceSide()));
    
    mAutonomousRoutines.add(new AutonomousEntry(currentIndex++, "Avoidance Shoot Close To Stage Close To Source Side", routineFactory.AvoidanceShootCloseToStageCloseToSourceSide()));
    mAutonomousRoutines.add(new AutonomousEntry(currentIndex++, "Former Playoffs A", routineFactory.PlayoffsA()));
    
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
    //SmartDashboard.putBoolean("Forward Limit", mShooter.testGetForwardAnglerLimitSwitchTriggered());
    //SmartDashboard.putBoolean("Reverse Limit", mShooter.testGetReverseAnglerLimitSwitchTriggered());


    if(mLoggingEnabled)
    {
      //mClimberArms.logArms();
      //SmartDashboard.putNumber("Logging At", System.currentTimeMillis());
      //mLimelightManager.logLimelightInfo();
      
      //mShooter.logShooterInformation();
      //mStateMachine.log();
      mExtendoArm.logExtendoArm();
      
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

    //ATS I don't think we're using this at all any more
    //mLimelightManager.calculateCameraPoseTargetSpace();
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
  
    mShooter.ConfigureShooterMotorsForGameMode(RobotMode.AUTONOMOUS);
    mSwerveDrive.ConfigureDriveMotorsForGameMode(true);
    
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
    SmartDashboard.putString("Alliance Info", AllianceInfo.GetInstance().GetName());
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

    mShooter.SetAnglerLimitSwitchesEnabledOrDisabled(true);

    mShooter.ConfigureShooterMotorsForGameMode(RobotMode.TELEOPERATED);
    mSwerveDrive.ConfigureDriveMotorsForGameMode(false);
    
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

    mShooter.ConfigureShooterMotorsForGameMode(RobotMode.TELEOPERATED);
    mSwerveDrive.ConfigureDriveMotorsForGameMode(false);

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
        //eh, let's just not make this possible.... too risky.     
        //mSwerveDrive.Run(mControls);
      }
      else
      {        
        mSwerveDrive.StopEverything();
      }
      
      SmartDashboard.putBoolean("Break Beam Status", mIntake.getBreakBeamStatus());
  }


  private void testSimpleExtendoArm()
  {
    double speed = mMainController.getRightTriggerAxis() - mMainController.getLeftTriggerAxis();
    if(Math.abs(speed) > 0.1)
    {
      mExtendoArm.testOnlyRunAtSpeed((mMainController.getRightTriggerAxis() - mMainController.getLeftTriggerAxis())/5.0);
    }
    else
    {
      mExtendoArm.testOnlyRunAtSpeed(0);
    }
    
  }

  /*
private void testPIDExtendoArm()
  {
    
    if(mMainController.getAButton())
    {
      mExtendoArm.testOnlyPIDToPosition(5);
    }
    else if(mMainController.getXButton())
    {
      mExtendoArm.testOnlyPIDToPosition(20);
    }
    else if(mMainController.getYButton())
    {
      mExtendoArm.testOnlyPIDToPosition(40);
    }
    else if(mMainController.getBButton())
    {
      mExtendoArm.testOnlyPIDToPosition(60);
    }
    else
    {
      mExtendoArm.testOnlyRunAtSpeed(0);
    }
    
  }
  */

  //private boolean mTestOnlyLastWasPressingMotionButton = false;

  public void testSmartExtendoArmMotion()
  {
    
    boolean isPressingSmartMotionButton = false;
    if(mMainController.getRightBumper())
    {
      if(mMainController.getAButton())
      {
        mExtendoArm.smartSetPosition(10);
        isPressingSmartMotionButton = true;
      }
      else if(mMainController.getXButton())
      {
        mExtendoArm.smartSetPosition(20);
        isPressingSmartMotionButton = true;
        
      }
      else if(mMainController.getYButton())
      {
        mExtendoArm.smartSetPosition(40);
        isPressingSmartMotionButton = true;
        
      }
      else if(mMainController.getBButton())
      {
        mExtendoArm.smartSetPosition(60);
        isPressingSmartMotionButton = true;
        
      }
      else
      {
        mExtendoArm.testOnlyRunAtSpeed(0);
      }

      /*
      if(isPressingSmartMotionButton && !mTestOnlyLastWasPressingMotionButton)
      {
        mExtendoArm.setSmartMotionInitial();
      }
      */
      if(isPressingSmartMotionButton)
      {
        mExtendoArm.continousSmartPositionUpdate();
      }
      //mTestOnlyLastWasPressingMotionButton = isPressingSmartMotionButton;
    }
    else
    {
      mExtendoArm.testOnlyRunAtSpeed(0);
      //mTestOnlyLastWasPressingMotionButton = false;
    }


  }
  
  public void testPeriodic()
  {    
    //testSimpleExtendoArm();
    //testPIDExtendoArm();
    //testSmartExtendoArmMotion();
    RealTestMode(); 

    
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

    //mExtendoArm.goToPosition(mExtendoTargetPosition);
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
