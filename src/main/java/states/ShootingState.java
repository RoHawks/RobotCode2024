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
import frc.robot.LimelightInformation;
import robosystems.Shooter;
import states.IntakingState.IntakeMode;
import robosystems.ExtendoArm;
import robosystems.Intake;

import universalSwerve.SwerveDrive;



public class ShootingState extends AState {
    private SwerveDrive mSwerveDrive;
    private Intake mIntake;
    private Shooter mShooter;
    private ExtendoArm mExtendoArm;
    private Controls mControls;

    private ShooterMode mShooterMode;
    private double mTimeStartedToShoot;

    private boolean mHasShot;

    public ShootingState(
        SwerveDrive pSwerveDrive,
        Intake pIntake,
        Shooter pShooter,
        ExtendoArm pExtendoArm,
        Controls pControls
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

    @Override
    public NextStateInfo Run() {
        // ATS commented out for tests!
        ChassisSpeeds chassisSpeeds = mSwerveDrive.getLastRequestedChassisSpeeds();

        logShootingStateInformation();
        checkForShootingPreperationButtons();
        mShooter.setAngleBasedOnShooterMode(mShooterMode, chassisSpeeds);
        
        

        if (mShooterMode == ShooterMode.LowGoal)
        {
            mSwerveDrive.Run(mControls, true, Constants.LOW_GOAL_ROTATION);
            mExtendoArm.lowGoalExtension();
            mShooter.spinUpToLowGoalSpeed();
        }
        else if (mShooterMode == ShooterMode.HighGoalManual)
        {
            mSwerveDrive.Run(mControls, true, Constants.HIGH_GOAL_ROTATION);
            mExtendoArm.retract();
            mShooter.spinUpToHighGoalSpeed();
        }
        else if (mShooterMode == ShooterMode.AutoAim)
        {
            mSwerveDrive.Run(mControls); // later whatever stuff I need to do
            mExtendoArm.retract();
            mShooter.spinUpToHighGoalSpeed();
        } 
        else if (mShooterMode == ShooterMode.HighGoalDriveBy)
        {
            mSwerveDrive.Run(mControls, true, Constants.HIGH_GOAL_ROTATION);
            mExtendoArm.retract();
            mShooter.spinUpToHighGoalSpeed();
        }

        boolean highGoalDriveByExtensionConditonsMet = true;
        boolean cameraInLeftRightRange = false;
        double value = -1;
        double adjustedCenter = -1;
        if (mShooterMode == ShooterMode.HighGoalDriveBy)
        {
            double[] cameraPositionInTargetSpace = LimelightInformation.getCameraPoseTargetSpace();
            if (cameraPositionInTargetSpace != null)
            {
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
                cameraInLeftRightRange = false;
                highGoalDriveByExtensionConditonsMet = false;
            }
        }
        
        boolean lowGoalExtensionConditionsMet = !(mShooterMode == ShooterMode.LowGoal && !mExtendoArm.hasReachedLowGoal());
       
        boolean persistentlyHasCorrectSpeed = mShooter.checkIfPersistentlyHasCorrectSpeed(mShooterMode);

        boolean specialTestVariableToEnableShooting = true;

        SmartDashboard.putBoolean("persistentlyHasCorrectSpeed", persistentlyHasCorrectSpeed);
        SmartDashboard.putBoolean("lowGoalExtensionConditionsMet", lowGoalExtensionConditionsMet);
        SmartDashboard.putBoolean("highGoalDriveByExtensionConditonsMet", highGoalDriveByExtensionConditonsMet);
        SmartDashboard.putBoolean("mHasShot", mHasShot);
        SmartDashboard.putBoolean("specialTestVariableToEnableShooting", specialTestVariableToEnableShooting);
        SmartDashboard.putBoolean("cameraInLeftRightRange", cameraInLeftRightRange);


       
        if (
            ((persistentlyHasCorrectSpeed
            && lowGoalExtensionConditionsMet
            && highGoalDriveByExtensionConditonsMet)
            || mHasShot) 
            && specialTestVariableToEnableShooting
            )
        {
            if (!mHasShot)
            {
                SmartDashboard.putNumber("DriveByTest: Distance when shot", value);
                SmartDashboard.putNumber("DriveByTest: Adjusted Center when shot", adjustedCenter);
                SmartDashboard.putNumber("DriveByTest: Distance From Adjusted Center When Shot", Math.abs(value - adjustedCenter));
                mHasShot = true;
                mTimeStartedToShoot = System.currentTimeMillis();
                
                if (mShooterMode == ShooterMode.HighGoalManual)
                {
                   
                }
                else if (mShooterMode == ShooterMode.LowGoal)
                {
                    mSwerveDrive.TurnAllWheels(270);
                }
            } 
            mIntake.setToLaunchingNoteIntoTheShooterSpeed();
        }
        else
        {
            mIntake.setToHoldingSpeed();
        }
        
        SmartDashboard.putString("ShooterMode", mShooterMode.name());
        
        
    
        if(mControls.GetForceIntakingMode())
        {
            return new NextStateInfo(States.Intaking, mShooterMode);
        }

        SmartDashboard.putNumber("Shooting: Time Elapsed Since Shooting", System.currentTimeMillis() - mTimeStartedToShoot);

        if (mHasShot && System.currentTimeMillis() - mTimeStartedToShoot > 500)
        {
            return new NextStateInfo(States.Intaking, mShooterMode);
        }
        else
        {
            return new NextStateInfo(States.Shooting, mShooterMode);
        }
        
    }

    private void checkForShootingPreperationButtons()
    {
        if(mControls.GetPrepareForHighGoalManual())
        {
            mShooterMode = ShooterMode.HighGoalManual;
        }
        else if(mControls.GetPrepareForHighGoalDriveBy())
        {
            mShooterMode = ShooterMode.HighGoalDriveBy;
        }
        else if(mControls.GetPrepareForLowGoal())
        {
            mShooterMode = ShooterMode.LowGoal;
        }
        else if(mControls.GetPrepareForAutoAim())
        {
            mShooterMode = ShooterMode.AutoAim;
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