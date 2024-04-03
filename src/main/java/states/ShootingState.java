package states;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.AllianceInfo;
import frc.robot.Constants;
import frc.robot.Functionality;
import frc.robot.LimelightInformation;
import frc.robot.LimelightManager;
import robosystems.Shooter;
import robosystems.Lights.LightingScheme;
import states.ClimbingModeManager.ClimbingMode;
import robosystems.ClimberArms;
import robosystems.ExtendoArm;
import robosystems.Intake;
import robosystems.Lights;
import universalSwerve.SwerveDrive;
import universalSwerve.utilities.AngleUtilities;



public class ShootingState extends AState {
    private SwerveDrive mSwerveDrive;
    private Intake mIntake;
    private Shooter mShooter;
    private ExtendoArm mExtendoArm;
    private Controls mControls;
    private Lights mLights;
    private ShooterMode mShooterMode;
    private double mTimeStartedToShoot;
    private LimelightManager mLimelightManager;
    private ClimberArms mClimberArms;

    private boolean mHasShot;

    public ShootingState(
        SwerveDrive pSwerveDrive,
        Intake pIntake,
        Shooter pShooter,
        ExtendoArm pExtendoArm,
        Controls pControls,
        Lights pLights,
        LimelightManager pLimelightManager,
        ClimberArms pClimberArms
        )
        {
            mSwerveDrive = pSwerveDrive;
            mIntake = pIntake;
            mShooter = pShooter;
            mControls = pControls;   
            mExtendoArm = pExtendoArm;
            mShooterMode = ShooterMode.HighGoalDriveBy;
            mTimeStartedToShoot = -1;
            mHasShot = false;
            mLights = pLights;
            mLimelightManager = pLimelightManager;
            mClimberArms = pClimberArms;
        }
       
        
    private double CENTER_POINT = -0.29;    
    //TOWNSEND: private double ADJUSTMENT_RATIO_FOR_DRIVING = ((0.2) / 4.5) * 5;
    private double ADJUSTMENT_RATIO_FOR_DRIVING = ((0.2) / 4.5) * 5.0 * 1.2;
    /**
     * Runs the intake. <br><br>
     * Ejection will only work while the button is held, and will go back to normal intaking if released
     */

    private double getAdjustedCenterFromChassisSpeed(ChassisSpeeds pChassisSpeeds)
    {

        //SmartDashboard.putNumber("pChassisSpeeds.vyMetersPerSecond", pChassisSpeeds.vyMetersPerSecond);
        return CENTER_POINT +  -(pChassisSpeeds.vyMetersPerSecond * ADJUSTMENT_RATIO_FOR_DRIVING);
    }

    private double getAdjustedCenterFromChassisSpeed_MegaTagBotPose(ChassisSpeeds pChassisSpeeds)
    {
        return AllianceInfo.GetInstance().GetCentralAprilTagDistanceFromDriversRightWall() +  -(pChassisSpeeds.vyMetersPerSecond * ADJUSTMENT_RATIO_FOR_DRIVING);
    }

    private double getLeftRightErrorToleranceFromChassisSpeeds(ChassisSpeeds pChassisSpeeds)
    {
        double absoluteSpeed = Math.abs(pChassisSpeeds.vyMetersPerSecond);
        //full speed:   Constants.DRIVE_BY_SHOOTNG_DISTANCE_ERROR_MARGIN
        //Stopped = 0.05;

        //public static final double DRIVE_BY_SHOOTNG_DISTANCE_ERROR_MARGIN = 0.15; // arbitrary
        //public static final double DRIVE_BY_SHOOTNG_DISTANCE_ERROR_MARGIN_WHEN_STOPPED = 0.04; // arbitrary

        double maxSpeedMetersPerSecond = 5.0;
        double percentageOfMaxSpeed = (absoluteSpeed) / maxSpeedMetersPerSecond;
        return Math.min(
            Constants.DRIVE_BY_SHOOTNG_DISTANCE_ERROR_MARGIN_WHEN_STOPPED + ((Constants.DRIVE_BY_SHOOTNG_DISTANCE_ERROR_MARGIN - Constants.DRIVE_BY_SHOOTNG_DISTANCE_ERROR_MARGIN_WHEN_STOPPED) * percentageOfMaxSpeed)
            , Constants.DRIVE_BY_SHOOTNG_DISTANCE_ERROR_MARGIN );            

    }


    private void basicContinousActions(ChassisSpeeds pChassisSpeeds)
    {
        Functionality.setArmsBasedOnClimberMode(ClimbingModeManager.getClimbingMode(), mClimberArms);
        // logShootingStateInformation();
        SmartDashboard.putString("ShooterMode", mShooterMode.name());
        ClimbingModeManager.determineClimbingMode(mControls);
        mShooterMode = Functionality.checkForShootingPreperationButtons(mControls, mShooterMode);        
        
        mShooter.checkIfPersistentlyHasCorrectSpeed(mShooterMode);
        // mShooter.logShooterInformation();
        if (mShooterMode != ShooterMode.AutoAim)
        {
            mShooter.setAngleBasedOnShooterMode(mShooterMode, pChassisSpeeds);
        }
        // setExtendoArmPosition();
        setShooterToShooterModesAppropriateSpeed();
        runSwerveDriveToShooterModesAppropriateBehavior();
        mExtendoArm.continousSmartPositionUpdate();
        
  


    }

    public static final boolean USE_MEGA_TAG_BOT_POSE = true;

    private double mAdjustedCenter = -1;
    private boolean mCameraInLeftRightRange = false;
    private boolean mAnglerInCorrectRange = false;
    private boolean mHighGoalDriveByConditonsMet = true;
    private boolean mSpecialTestVariableToEnableShooting = true;
    private boolean mPersistentlyHasCorrectSpeed = false;

    private boolean isFirstTimeRunningAutoAimCalculations = true;
    private double mRecordedTurningAngle = 0;
    private double mTimeWhenCloseEnough = -1;

    public static boolean TECH_VALLEY_AUTO_SHOOTING_VERSION = false;
    private int mReAimCount = 0;

    @Override
    public NextStateInfo Run() {
        ShooterMode initialShooterMode = mShooterMode;
        ChassisSpeeds chassisSpeeds = mSwerveDrive.getLastRequestedChassisSpeeds();
        basicContinousActions(chassisSpeeds);
        if ((initialShooterMode == ShooterMode.HighGoalDriveBy && mShooterMode == ShooterMode.HighGoalManual)
            || inAutoModeAndCouldntSeeTheTagAtTheStart)
        {
            return new NextStateInfo(States.Holding, mShooterMode);
        }

        if ((initialShooterMode == ShooterMode.HighGoalDriveBy && mShooterMode == ShooterMode.AutoAim))
        {
            return new NextStateInfo(States.Holding, mShooterMode);
        }

        //ATS, 3/19:  adding the code below
        //If you switched to low goal modew while it was in drive-by, it would just shoot right away.

        if(initialShooterMode != ShooterMode.LowGoal && mShooterMode == ShooterMode.LowGoal)
        {
            return new NextStateInfo(States.Holding, mShooterMode);
        }

        if (mShooterMode == ShooterMode.AutoAim && isFirstTimeRunningAutoAimCalculations)
        {
            mShooter.setAngleBasedOnShooterMode(mShooterMode);
            mTimeWhenCloseEnough = -1;
            isFirstTimeRunningAutoAimCalculations = false;
        }

        if (mShooterMode == ShooterMode.LowGoal || mShooterMode == ShooterMode.HighGoalManual)
        {
            mLights.SetLightingScheme(LightingScheme.HoldingButNoCameraLock);
        }
      

        boolean autoShootingConditionsMet = false;
        if (mShooterMode == ShooterMode.AutoAim)
        {
            if(TECH_VALLEY_AUTO_SHOOTING_VERSION)
            {
                double gyroAngle = AngleUtilities.Normalize(mSwerveDrive.GetGyroscopeCurrentAngle()); 
                
                SmartDashboard.putNumber("mRecordedTurningAngle", mRecordedTurningAngle);
                SmartDashboard.putNumber("gyroAngle", gyroAngle);
                
                boolean closeEnough = AngleUtilities.AbsoluteDistanceBetweenAngles(gyroAngle, mRecordedTurningAngle) < 7;

                if (closeEnough && mTimeWhenCloseEnough < 0)
                {
                    SmartDashboard.putNumber("AutoLogs Before: mTimeWhenCloseEnough", mTimeWhenCloseEnough);
                    mTimeWhenCloseEnough = System.currentTimeMillis();
                }
                else if (!closeEnough)
                {
                    mTimeWhenCloseEnough = -1;
                }

                SmartDashboard.putNumber("AutoLogs: mTimeWhenCloseEnough Difference", System.currentTimeMillis() -  mTimeWhenCloseEnough);
                SmartDashboard.putNumber("AutoLogs After: mTimeWhenCloseEnough", mTimeWhenCloseEnough);
                SmartDashboard.putBoolean("AutoLogs: AnglerCloseEnough", mShooter.checkIfAnglerIsCloseEnough());
                if (mTimeWhenCloseEnough > 0 && System.currentTimeMillis() - mTimeWhenCloseEnough > 200)//Ats was1000 changed to 200
                {
                    // mShooter.checkIfPersistentlyHasCorrectSpeed(ShooterMode.HighGoalManual);
                    // double angle = LimelightInformation.GetAngleToAprilTag(mLimelightManager.getBotPoseByPriorityCamera(LimelightManager.EAST_CAMERA));
                    if (mShooter.checkIfAnglerIsCloseEnough()) // gyroAngle == angle &&
                    {
                        SmartDashboard.putNumber("AutoLog: is Shooting", System.currentTimeMillis());    
                        autoShootingConditionsMet = true;
                    }
                    else 
                    {
                    // recordedTurningAngle = LimelightInformation.GetAngleToAprilTag(mLimelightManager.getBotPoseByPriorityCamera(LimelightManager.EAST_CAMERA));
                    // angleShooterToTheAprilTag();
                    }
                }
                else
                {
                    mShooter.spinUpToHighGoalSpeed();
                    mIntake.setToHoldingSpeed();
                }
            }
            else //New version for NY Regional
            {
                double gyroAngle = AngleUtilities.Normalize(mSwerveDrive.GetGyroscopeCurrentAngle()); 
                
                SmartDashboard.putNumber("mRecordedTurningAngle", mRecordedTurningAngle);
                SmartDashboard.putNumber("gyroAngle", gyroAngle);
                
                boolean closeEnough = AngleUtilities.AbsoluteDistanceBetweenAngles(gyroAngle, mRecordedTurningAngle) < 7;
                
                
                

                if (closeEnough && mTimeWhenCloseEnough < 0)
                {
                    SmartDashboard.putNumber("AutoLogs Before: mTimeWhenCloseEnough", mTimeWhenCloseEnough);
                    mTimeWhenCloseEnough = System.currentTimeMillis();
                }
                else if (!closeEnough)
                {
                    mTimeWhenCloseEnough = -1;
                }

                SmartDashboard.putNumber("AutoLogs: mTimeWhenCloseEnough Difference", System.currentTimeMillis() -  mTimeWhenCloseEnough);
                SmartDashboard.putNumber("AutoLogs After: mTimeWhenCloseEnough", mTimeWhenCloseEnough);
                SmartDashboard.putBoolean("AutoLogs: AnglerCloseEnough", mShooter.checkIfAnglerIsCloseEnough());
                if (mTimeWhenCloseEnough > 0 && System.currentTimeMillis() - mTimeWhenCloseEnough > 200)//Ats was1000 changed to 200
                {
                    if(mReAimCount > 0)
                    {
                        //I've already tried to reaim once.                          
                        //I think just fire it!
                        autoShootingConditionsMet = true;
                    }
                    else
                    {
                        if (mShooter.checkIfAnglerIsCloseEnough()) // gyroAngle == angle &&
                        {

                            //ATS:  We've now reached where we originally wanted to be.
                            //But for NY, we're going to double check now to see if we are, in fact, still close enough
                        
                            //First lets check if we can still see the tag.
                            double[] newCameraData = mLimelightManager.getBotPoseByPriorityCamera(LimelightManager.EAST_CAMERA);
                            boolean canSeeTag = LimelightInformation.isValidBotPoseResults(newCameraData);
                            if(!canSeeTag)
                            {
                                //We could see it when we first hit the trigger, but now we can't.... let's just go ahead and fire!
                                autoShootingConditionsMet = true;
                            }
                            else
                            {
                                //We could see it when we pulled the trigger, 
                                //we turned to where we think we should be
                                //and we can still see it.  
                                //Let's figure out if we're still close enough.
                                mRecordedTurningAngle = this.GetGoalTurningAngle();
                                boolean gyroSecondCheckCloseEnough = AngleUtilities.AbsoluteDistanceBetweenAngles(gyroAngle, mRecordedTurningAngle) < 7;
                                double anglerTargetAngle = mShooter.calculateAutoAngleWithRotations_MegaTagBotPose_WithCameraPreference(LimelightManager.EAST_CAMERA);
                                boolean anglerSecondCheckCloseEnough = mShooter.checkIfAnglerIsCloseEnough(anglerTargetAngle);

                                if(gyroSecondCheckCloseEnough && anglerSecondCheckCloseEnough )
                                {
                                    autoShootingConditionsMet = true;
                                }
                                else
                                {
                                    mShooter.setAngle(anglerTargetAngle);
                                    mTimeWhenCloseEnough = -1;//reset things and let's try this again.
                                    mReAimCount++;
                                    //Maybe put a limit to how many times we try this before just shooting???
                                }
                            }

                        }
                        else 
                        {
                        // recordedTurningAngle = LimelightInformation.GetAngleToAprilTag(mLimelightManager.getBotPoseByPriorityCamera(LimelightManager.EAST_CAMERA));
                        // angleShooterToTheAprilTag();
                        }
                    }
                }
                else
                {
                    mShooter.spinUpToHighGoalSpeed();
                    mIntake.setToHoldingSpeed();
                }
            }
        }
        else
        {
            autoShootingConditionsMet = true;
        }
        

        // else if (initialShooterMode != ShooterMode.LowGoal && mShooterMode == ShooterMode.LowGoal) // hacky way to make the note not shoot if you swap to manual midmode
        // {
        //     return new NextStateInfo(States.Holding, mShooterMode);
        // }

        
        
        
        //SmartDashboard.putString("Current Shooter Mode", mShooterMode.name());
        mHighGoalDriveByConditonsMet = false;
        if (mShooterMode == ShooterMode.HighGoalDriveBy)
        {
            if(USE_MEGA_TAG_BOT_POSE)
            {
                /*ATS, let's try out the bot pose mega tag thing to see if we get more consistent results */
            
            //too tired, need to skip this for now.
            double[] botposeMegaTag = mLimelightManager.getBotPoseFromCameraBasedOnChassisSpeeds(chassisSpeeds);
            mHighGoalDriveByConditonsMet = isHighGoalDriveByConditionsMet_MegaTagBotPoseVersion(chassisSpeeds, botposeMegaTag);
            setLightsInDriveByMode(botposeMegaTag);
            

            }
            else
            {
                //Older version which use the just one tag            
                double[] cameraPositionInTargetSpace = mLimelightManager.getCameraPoseTargetSpace();
                mHighGoalDriveByConditonsMet = isHighGoalDriveByConditionsMet(chassisSpeeds, cameraPositionInTargetSpace);
                setLightsInDriveByMode(cameraPositionInTargetSpace);
            }

            

        }
        else
        {
            mHighGoalDriveByConditonsMet = true;
        }

        // mAutoAimConditonsMet = false;
        // if (mShooterMode == ShooterMode.AutoAim)
        // {
        //     double[] cameraPositionInTargetSpace = mLimelightManager.getCameraPoseTargetSpace();
        //     mAutoAimConditonsMet = isAutoAimConditionsMet(chassisSpeeds, cameraPositionInTargetSpace);

        // }
        // else
        // {
        //     mAutoAimConditonsMet = true;
        // }
        
        boolean lowGoalExtensionConditionsMet = !(mShooterMode == ShooterMode.LowGoal && !mExtendoArm.hasReachedLowGoal());
        mPersistentlyHasCorrectSpeed = mShooter.checkIfPersistentlyHasCorrectSpeed(mShooterMode);
        mSpecialTestVariableToEnableShooting = true;

        boolean generalConditionsMet = mPersistentlyHasCorrectSpeed && 
                lowGoalExtensionConditionsMet && 
                mHighGoalDriveByConditonsMet &&
                autoShootingConditionsMet;

        SmartDashboard.putBoolean("Conditions: mPersistentlyHasCorrectSpeed", mPersistentlyHasCorrectSpeed);
        SmartDashboard.putBoolean("Conditions: lowGoalExtensionConditionsMet", lowGoalExtensionConditionsMet);
        SmartDashboard.putBoolean("Conditions: mHighGoalDriveByConditonsMet", mHighGoalDriveByConditonsMet);
        SmartDashboard.putBoolean("Conditions: autoShootingConditionsMet", autoShootingConditionsMet);
        

        boolean readyToShoot = (generalConditionsMet || mHasShot) && mSpecialTestVariableToEnableShooting;


       
        if (readyToShoot)
        {
            if (!mHasShot)
            {
                mHasShot = true;
                mTimeStartedToShoot = System.currentTimeMillis();
            } 
            mLights.SetLightingScheme(LightingScheme.Shooting);
            mIntake.setToLaunchingNoteIntoTheShooterSpeed();
        }
        else
        {
            mIntake.setToHoldingSpeed();
        }
        
        mLights.Run();
    
        
        if (mControls.GetForceEjectionMode())
        {
            return new NextStateInfo(States.Ejecting, mShooterMode);
        }
        else if (mControls.GetForceIntakingMode())
        {
            return new NextStateInfo(States.Intaking, mShooterMode);
        }

        double timeDelay;
        if (mShooterMode == ShooterMode.LowGoal)
        {
            timeDelay = 2500;
        }
        else
        {
            timeDelay = 500;
        }

        if (mHasShot && mShooterMode == ShooterMode.LowGoal)
        {
            if (System.currentTimeMillis() - mTimeStartedToShoot > 500)
            {
                // SmartDashboard.putNumber()
                mExtendoArm.retract();
            }
        }
        else
        {
            setExtendoArmPosition();
        }

        if (mControls.GetRetractClimb() && ClimbingModeManager.getClimbingMode() == ClimbingMode.Extending)
        {
            return new NextStateInfo(States.Climbing, mShooterMode);
        }
        
        if (mHasShot && System.currentTimeMillis() - mTimeStartedToShoot > timeDelay)
        {
            return new NextStateInfo(States.Intaking, mShooterMode);
        }
        else
        {
            return new NextStateInfo(States.Shooting, mShooterMode);
        }
        
    }

    /*
    double mPlaceHolderForLimeLightX;
    private void logDriveByShootingInformationGeneral()
    {
        mPlaceHolderForLimeLightX = -1;
        double[] cameraPositionInTargetSpace = mLimelightManager.getCameraPoseTargetSpace();
        if (cameraPositionInTargetSpace != null)
        {
            mPlaceHolderForLimeLightX = cameraPositionInTargetSpace[0];
            //SmartDashboard.putNumber("Lime: X Displacement", mPlaceHolderForLimeLightX);
        }

        boolean lowGoalExtensionConditionsMet = !(mShooterMode == ShooterMode.LowGoal && !mExtendoArm.hasReachedLowGoal());

        
    }
    */
    /*
    private void logDriveByShootingInformationWhenShot()
    {
        
        SmartDashboard.putNumber("DriveByTest: Distance when shot", mPlaceHolderForLimeLightX);
        SmartDashboard.putNumber("DriveByTest: Adjusted Center when shot", mAdjustedCenter);
        SmartDashboard.putNumber("DriveByTest: Distance From Adjusted Center When Shot", Math.abs(mPlaceHolderForLimeLightX - mAdjustedCenter));
        SmartDashboard.putNumber("DriveByTest: Adjusted Center Point when shot", mAdjustedCenter);      
    }
    */

    private double mCalculatedDistanceFromEastFieldWall = 0;
    private boolean isHighGoalDriveByConditionsMet_MegaTagBotPoseVersion(ChassisSpeeds pChassisSpeeds, double[] pBotpose)
    {
        mAdjustedCenter = -1;
        mCameraInLeftRightRange = false;
        if (pBotpose != null && LimelightInformation.isValidBotPoseResults(pBotpose))
        {

            if (pChassisSpeeds == null)
            {
                mAdjustedCenter = AllianceInfo.GetInstance().GetCentralAprilTagDistanceFromDriversRightWall();
            }
            else
            {
                mAdjustedCenter = getAdjustedCenterFromChassisSpeed_MegaTagBotPose(pChassisSpeeds);
            }
            
            double leftRightErrorToleranceFromChassisSpeeds = getLeftRightErrorToleranceFromChassisSpeeds(pChassisSpeeds);
            mCalculatedDistanceFromEastFieldWall = LimelightInformation.BotPose_GetDistanceFromEastFieldWall(pBotpose);
            mCameraInLeftRightRange = Math.abs(mCalculatedDistanceFromEastFieldWall - mAdjustedCenter) < leftRightErrorToleranceFromChassisSpeeds; //Old version: Constants.DRIVE_BY_SHOOTNG_DISTANCE_ERROR_MARGIN;                 
            mAnglerInCorrectRange = mShooter.getIfAutomaticAnglerInRange(pChassisSpeeds);
            mHighGoalDriveByConditonsMet = mCameraInLeftRightRange &&  mAnglerInCorrectRange;
            // highGoalDriveByExtensionConditonsMet = cameraInLeftRightRange; //for now ignore height of shooter && mShooter.getIfAutomaticAnglerInRange();
            
        }
        else
        {
            mCameraInLeftRightRange = false;
            mHighGoalDriveByConditonsMet = false;
        }
        return mHighGoalDriveByConditonsMet;

    }


    private boolean isHighGoalDriveByConditionsMet(ChassisSpeeds pChassisSpeeds, double[] pCameraPositionInTargetSpace)
    {
        mAdjustedCenter = -1;
        mCameraInLeftRightRange = false;
        if (pCameraPositionInTargetSpace != null)
        {

            if (pChassisSpeeds == null)
            {
                mAdjustedCenter = CENTER_POINT;
            }
            else
            {
                mAdjustedCenter = getAdjustedCenterFromChassisSpeed(pChassisSpeeds);
            }
            
            double leftRightErrorToleranceFromChassisSpeeds = getLeftRightErrorToleranceFromChassisSpeeds(pChassisSpeeds);

            mCameraInLeftRightRange = Math.abs(pCameraPositionInTargetSpace[0] - mAdjustedCenter) < leftRightErrorToleranceFromChassisSpeeds; //Old version: Constants.DRIVE_BY_SHOOTNG_DISTANCE_ERROR_MARGIN;                 
            mAnglerInCorrectRange = mShooter.getIfAutomaticAnglerInRange(pChassisSpeeds);
            mHighGoalDriveByConditonsMet = mCameraInLeftRightRange &&  mAnglerInCorrectRange;
            // highGoalDriveByExtensionConditonsMet = cameraInLeftRightRange; //for now ignore height of shooter && mShooter.getIfAutomaticAnglerInRange();
            
        }
        else
        {
            mCameraInLeftRightRange = false;
            mHighGoalDriveByConditonsMet = false;
        }
        return mHighGoalDriveByConditonsMet;
    }

    private void setExtendoArmPosition()
    {
        if (mShooterMode == ShooterMode.LowGoal)
        {
            mExtendoArm.lowGoalExtension();
        }
        else
        {
           mExtendoArm.retract(); 
        }
    }

    private void setShooterToShooterModesAppropriateSpeed()
    {
        if (mShooterMode == ShooterMode.LowGoal)
        {
            mShooter.spinUpToLowGoalSpeed();
        }
        else
        {
           mShooter.spinUpToHighGoalSpeed();
        }
    }

    private void runSwerveDriveToShooterModesAppropriateBehavior()
    {
        if (mShooterMode == ShooterMode.LowGoal)
        {
            mSwerveDrive.Run(mControls, true, Constants.LOW_GOAL_ROTATION);
        }
        else if (mShooterMode == ShooterMode.HighGoalDriveBy)
        {
            mSwerveDrive.Run(mControls, true, Constants.HIGH_GOAL_ROTATION);
        }
        else if (mShooterMode == ShooterMode.AutoAim)
        {
            mSwerveDrive.Run(mControls, true, mRecordedTurningAngle);
        }
        else 
        {
            mSwerveDrive.Run(mControls); 
        }

    }

    private void setLightsInDriveByMode(double[] pCameraPositionInTargetSpace)
    {
         
        if (!mHasShot)
        {
            if (pCameraPositionInTargetSpace != null && LimelightInformation.isValidBotPoseResults(pCameraPositionInTargetSpace))
            {
                mLights.SetLightingScheme(LightingScheme.HoldingWithCameraLock);
            }
            else
            {
                mLights.SetLightingScheme(LightingScheme.HoldingButNoCameraLock);
            }
        }
    }

    private boolean inAutoModeAndCouldntSeeTheTagAtTheStart = false;
    protected void EnterState(Object pEntryParameter)
    {
        super.EnterState(pEntryParameter);
        //For normal usage
        mShooterMode = (ShooterMode) pEntryParameter;
        mTimeStartedToShoot = -1;
        
        mHasShot = false;
        mReAimCount = 0;
        mShooter.ResetNonCameraAutoAimAngle();
        mControls.TurnOffVibrate();

        isFirstTimeRunningAutoAimCalculations = true;
        inAutoModeAndCouldntSeeTheTagAtTheStart = false;
        if (mShooterMode == ShooterMode.AutoAim)
        {
            //add stuff for if its null
            boolean canSeeTag = LimelightInformation.isValidBotPoseResults(mLimelightManager.getBotPoseByPriorityCamera(LimelightManager.EAST_CAMERA));

            if (!canSeeTag)
            {
                inAutoModeAndCouldntSeeTheTagAtTheStart = true;
                return;
            }
            else
            {
                inAutoModeAndCouldntSeeTheTagAtTheStart = false;
            }
            
            mRecordedTurningAngle = GetGoalTurningAngle();
            SmartDashboard.putNumber("Auto Log: mRecordedTurningAngle", mRecordedTurningAngle);
        }
        
    }

    private double GetGoalTurningAngle()
    {
        return LimelightInformation.GetAngleToAprilTag(mLimelightManager.getBotPoseByPriorityCamera(LimelightManager.EAST_CAMERA));
    }

    @Override
    protected States GetState() {
        return States.Shooting;
    }

    @Override
    protected String GetName() {
        return "Shooting";
    }

    @Override
    protected void logStateInfo()
    {
        SmartDashboard.putString("Shooting: Shooting Mode", mShooterMode.name());
        SmartDashboard.putBoolean("Shooting: Persistently at Speed", mPersistentlyHasCorrectSpeed);
        SmartDashboard.putBoolean("Shooting: mHasShot", mHasShot);        
        SmartDashboard.putBoolean("Shooting: highGoalDriveByExtensionConditonsMet", mHighGoalDriveByConditonsMet);
        SmartDashboard.putBoolean("Shooting: cameraInLeftRightRange", mCameraInLeftRightRange);        
        SmartDashboard.putNumber("Shooting: Time Elapsed Since Shooting", System.currentTimeMillis() - mTimeStartedToShoot);     
        SmartDashboard.putBoolean("Shooting: mAnglerInCorrectRange", mAnglerInCorrectRange);
        SmartDashboard.putNumber("Shooting: mAdjustedCenter", mAdjustedCenter);
        SmartDashboard.putNumber("Shooting: mCalculatedDistanceFromEastFieldWall", mCalculatedDistanceFromEastFieldWall);
            

    }
    
}