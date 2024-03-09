package states;

import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.networktables.NetworkTableValue;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import frc.robot.Constants;
import frc.robot.Functionality;
import frc.robot.LimelightManager;
import robosystems.Shooter;
import robosystems.Lights.LightingScheme;
import robosystems.ExtendoArm;
import robosystems.Intake;
import robosystems.Lights;
import universalSwerve.SwerveDrive;



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

    private boolean mHasShot;

    public ShootingState(
        SwerveDrive pSwerveDrive,
        Intake pIntake,
        Shooter pShooter,
        ExtendoArm pExtendoArm,
        Controls pControls,
        Lights pLights,
        LimelightManager pLimelightManager
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

        SmartDashboard.putNumber("pChassisSpeeds.vyMetersPerSecond", pChassisSpeeds.vyMetersPerSecond);
        return CENTER_POINT +  -(pChassisSpeeds.vyMetersPerSecond * ADJUSTMENT_RATIO_FOR_DRIVING);
    }

    private double getLeftRightErrorToleranceFromChassisSpeeds(ChassisSpeeds pChassisSpeeds)
    {
        double absoluteSpeed = Math.abs(pChassisSpeeds.vyMetersPerSecond);
        //full speed:   Constants.DRIVE_BY_SHOOTNG_DISTANCE_ERROR_MARGIN
        //Stopped = 0.05;

        double maxSpeedMetersPerSecond = 5.0;
        double percentageOfMaxSpeed = (absoluteSpeed) / maxSpeedMetersPerSecond;
        return Math.min(
            Constants.DRIVE_BY_SHOOTNG_DISTANCE_ERROR_MARGIN_WHEN_STOPPED + ((Constants.DRIVE_BY_SHOOTNG_DISTANCE_ERROR_MARGIN - Constants.DRIVE_BY_SHOOTNG_DISTANCE_ERROR_MARGIN_WHEN_STOPPED) * percentageOfMaxSpeed)
            , Constants.DRIVE_BY_SHOOTNG_DISTANCE_ERROR_MARGIN );            

    }


    private void basicContinousActions(ChassisSpeeds pChassisSpeeds)
    {
        // logShootingStateInformation();
        
        mShooterMode = Functionality.checkForShootingPreperationButtons(mControls, mShooterMode);        
        
        mShooter.checkIfPersistentlyHasCorrectSpeed(mShooterMode);
        // mShooter.logShooterInformation();
        mShooter.setAngleBasedOnShooterMode(mShooterMode, pChassisSpeeds);
        setExtendoArmPosition();
        setShooterToShooterModesAppropriateSpeed();
        runSwerveDriveToShooterModesAppropriateBehavior();
        mLights.Run();
  


    }

    double mAdjustedCenter = -1;
    boolean mCameraInLeftRightRange = false;
    boolean mHighGoalDriveByConditonsMet = true;
    boolean mSpecialTestVariableToEnableShooting = true;
    @Override
    public NextStateInfo Run() {
        
        ChassisSpeeds chassisSpeeds = mSwerveDrive.getLastRequestedChassisSpeeds();
        ShooterMode initialShooterMode = mShooterMode;
        basicContinousActions(chassisSpeeds);
        if (initialShooterMode != ShooterMode.HighGoalManual && mShooterMode == ShooterMode.HighGoalDriveBy) // hacky way to make the note not shoot if you swap to manual midmode
        {
            return new NextStateInfo(States.Holding, mShooterMode);
        }
        else if (initialShooterMode != ShooterMode.LowGoal && mShooterMode == ShooterMode.LowGoal) // hacky way to make the note not shoot if you swap to manual midmode
        {
            return new NextStateInfo(States.Holding, mShooterMode);
        }


        if (mShooterMode == ShooterMode.LowGoal || mShooterMode == ShooterMode.HighGoalManual)
        {
            mLights.SetLightingScheme(LightingScheme.HoldingButNoCameraLock);
        }
      
        
        mHighGoalDriveByConditonsMet = false;
        if (mShooterMode == ShooterMode.HighGoalDriveBy)
        {
<<<<<<< Updated upstream
            double[] cameraPositionInTargetSpace = mLimelightManager.getCameraPoseTargetSpace();
            mHighGoalDriveByConditonsMet = isHighGoalDriveByConditionsMet(chassisSpeeds, cameraPositionInTargetSpace);
            setLightsInDriveByMode(cameraPositionInTargetSpace);
=======
            double[] cameraPositionInTargetSpace = mLimelightManager.getCameraPoseTargetSpaceForSpecificCamera(LimelightManager.EAST_CAMERA);
            if (cameraPositionInTargetSpace != null)
            {
                if(!mHasShot)
                {
                    mLights.SetLightingScheme(LightingScheme.HoldingWithCameraLock);
                }
                value = cameraPositionInTargetSpace[0];
                SmartDashboard.putNumber("Lime: X Displacement", value);

                
                
                if (chassisSpeeds == null)
                {
                    adjustedCenter = CENTER_POINT;
                }
                else
                {
                    adjustedCenter = getAdjustedCenterFromChassisSpeed(chassisSpeeds);
                }

                SmartDashboard.putNumber("Adjusted Center Point", adjustedCenter);

                double leftRightErrorToleranceFromChassisSpeeds = getLeftRightErrorToleranceFromChassisSpeeds(chassisSpeeds);

                cameraInLeftRightRange = Math.abs(cameraPositionInTargetSpace[0] - adjustedCenter) < leftRightErrorToleranceFromChassisSpeeds; //Old version: Constants.DRIVE_BY_SHOOTNG_DISTANCE_ERROR_MARGIN;                 
                boolean isAnglerInCorrectRange = mShooter.getIfAutomaticAnglerInRange(chassisSpeeds);
                highGoalDriveByExtensionConditonsMet = cameraInLeftRightRange &&  isAnglerInCorrectRange;
                // highGoalDriveByExtensionConditonsMet = cameraInLeftRightRange; //for now ignore height of shooter && mShooter.getIfAutomaticAnglerInRange();
                
            }
            else
            {
                if(!mHasShot)
                {
                    mLights.SetLightingScheme(LightingScheme.HoldingButNoCameraLock);
                }
                cameraInLeftRightRange = false;
                highGoalDriveByExtensionConditonsMet = false;
            }
>>>>>>> Stashed changes
        }
        
        boolean lowGoalExtensionConditionsMet = !(mShooterMode == ShooterMode.LowGoal && !mExtendoArm.hasReachedLowGoal());
        boolean persistentlyHasCorrectSpeed = mShooter.checkIfPersistentlyHasCorrectSpeed(mShooterMode);
        mSpecialTestVariableToEnableShooting = true;

        boolean generalConditionsMet = persistentlyHasCorrectSpeed && lowGoalExtensionConditionsMet && mHighGoalDriveByConditonsMet;
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
        
    
        
        if (mControls.GetForceEjectionMode())
        {
            return new NextStateInfo(States.Ejecting, mShooterMode);
        }
        else if (mControls.GetForceIntakingMode())
        {
            return new NextStateInfo(States.Intaking, mShooterMode);
        }

        if (mHasShot && System.currentTimeMillis() - mTimeStartedToShoot > 500)
        {
            return new NextStateInfo(States.Intaking, mShooterMode);
        }
        else
        {
            return new NextStateInfo(States.Shooting, mShooterMode);
        }
        
    }

    double mPlaceHolderForLimeLightX;
    private void logDriveByShootingInformationGeneral()
    {
        mPlaceHolderForLimeLightX = -1;
        double[] cameraPositionInTargetSpace = mLimelightManager.getCameraPoseTargetSpace();
        if (cameraPositionInTargetSpace != null)
        {
            mPlaceHolderForLimeLightX = cameraPositionInTargetSpace[0];
            SmartDashboard.putNumber("Lime: X Displacement", mPlaceHolderForLimeLightX);
        }

        boolean lowGoalExtensionConditionsMet = !(mShooterMode == ShooterMode.LowGoal && !mExtendoArm.hasReachedLowGoal());
        SmartDashboard.putBoolean("persistentlyHasCorrectSpeed", mShooter.checkIfPersistentlyHasCorrectSpeed(mShooterMode));
        SmartDashboard.putBoolean("mHasShot", mHasShot);
        SmartDashboard.putBoolean("lowGoalExtensionConditionsMet", lowGoalExtensionConditionsMet);
        SmartDashboard.putBoolean("highGoalDriveByExtensionConditonsMet", mHighGoalDriveByConditonsMet);
        SmartDashboard.putBoolean("cameraInLeftRightRange", mCameraInLeftRightRange);
        SmartDashboard.putBoolean("specialTestVariableToEnableShooting", mSpecialTestVariableToEnableShooting);
        SmartDashboard.putNumber("Shooting: Time Elapsed Since Shooting", System.currentTimeMillis() - mTimeStartedToShoot);
        SmartDashboard.putString("ShooterMode", mShooterMode.name());
        
    }

    private void logDriveByShootingInformationWhenShot()
    {
        SmartDashboard.putNumber("DriveByTest: Distance when shot", mPlaceHolderForLimeLightX);
        SmartDashboard.putNumber("DriveByTest: Adjusted Center when shot", mAdjustedCenter);
        SmartDashboard.putNumber("DriveByTest: Distance From Adjusted Center When Shot", Math.abs(mPlaceHolderForLimeLightX - mAdjustedCenter));
        SmartDashboard.putNumber("DriveByTest: Adjusted Center Point when shot", mAdjustedCenter);
        
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
            boolean isAnglerInCorrectRange = mShooter.getIfAutomaticAnglerInRange(pChassisSpeeds);
            mHighGoalDriveByConditonsMet = mCameraInLeftRightRange &&  isAnglerInCorrectRange;
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
        else if (mShooterMode == ShooterMode.HighGoalManual || mShooterMode == ShooterMode.HighGoalDriveBy)
        {
            mSwerveDrive.Run(mControls, true, Constants.HIGH_GOAL_ROTATION);
        }
        else // if (mShooterMode == ShooterMode.AutoAim) is the only other possible case
        {
            mSwerveDrive.Run(mControls); 
        }

    }

    private void setLightsInDriveByMode(double[] pCameraPositionInTargetSpace)
    {
        if (!mHasShot)
        {
            if (pCameraPositionInTargetSpace != null)
            {
                mLights.SetLightingScheme(LightingScheme.HoldingWithCameraLock);
            }
            else
            {
                mLights.SetLightingScheme(LightingScheme.HoldingButNoCameraLock);
            }
        }
    }

    public void logShootingStateInformation()
    {
        SmartDashboard.putString("Shooting: Shooting Mode", mShooterMode.name());
        SmartDashboard.putBoolean("Persistently at Speed", mShooter.checkIfPersistentlyHasCorrectSpeed(mShooterMode));
        mShooter.logShooterInformation();
    }
  

    protected void EnterState(Object pEntryParameter)
    {
        super.EnterState(pEntryParameter);
        //For normal usage
        mShooterMode = (ShooterMode) pEntryParameter;
        mTimeStartedToShoot = -1;
        
        mHasShot = false;
    }

    

    @Override
    protected States GetState() {
        return States.Shooting;
    }

    @Override
    protected String GetName() {
        return "Shooting";
    }
    
}