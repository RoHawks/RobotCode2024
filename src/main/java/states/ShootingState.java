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
    private double ADJUSTMENT_RATIO_FOR_DRIVING = ((0.2) / 4.5) * 5;
    /**
     * Runs the intake. <br><br>
     * Ejection will only work while the button is held, and will go back to normal intaking if released
     */

    private double getAdjustedCenterFromChassisSpeed(ChassisSpeeds pChassisSpeeds)
    {
        return CENTER_POINT +  -(pChassisSpeeds.vyMetersPerSecond * ADJUSTMENT_RATIO_FOR_DRIVING);
    }

    @Override
    public NextStateInfo Run() {
        // ATS commented out for tests!
        
        logShootingStateInformation();
        checkForShootingPreperationButtons();
        mShooter.setAngleBasedOnShooterMode(mShooterMode);
        
        

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

        double value = -1;
        if (mShooterMode == ShooterMode.HighGoalDriveBy)
        {
            
            try
            {
                NetworkTable table = NetworkTableInstance.getDefault().getTable("limelight");
                NetworkTableEntry jsonNTE = table.getEntry("json");
                double[] cameraPositionInTargetSpace = Functionality.getCameraPoseTargetSpace(jsonNTE.getString(null));  
                value = cameraPositionInTargetSpace[0];
                SmartDashboard.putNumber("Continous Value", value);

                double adjustedCenter;
                ChassisSpeeds chassisSpeeds = mSwerveDrive.getLastRequestedChassisSpeeds();
                if (chassisSpeeds == null)
                {
                    adjustedCenter = CENTER_POINT;
                }
                else
                {
                    adjustedCenter = getAdjustedCenterFromChassisSpeed(chassisSpeeds);
                }

                SmartDashboard.putNumber("Adjusted Center Point", adjustedCenter);

                highGoalDriveByExtensionConditonsMet = Math.abs(cameraPositionInTargetSpace[0] - adjustedCenter) < Constants.DRIVE_BY_SHOOTNG_DISTANCE_ERROR_MARGIN; 
                highGoalDriveByExtensionConditonsMet = highGoalDriveByExtensionConditonsMet && mShooter.getIfAutomaticAnglerInRange();

            }
            catch (Exception e)
            {
                SmartDashboard.putNumber("Error 2",System.currentTimeMillis());
                highGoalDriveByExtensionConditonsMet = false;
            }// double distanceInXAndZ = Math.sqrt(
        }

        
        boolean lowGoalExtensionConditionsMet = !(mShooterMode == ShooterMode.LowGoal && !mExtendoArm.hasReachedLowGoal());
       

        boolean specialTestVariableToEnableShooting = true;
        if (
            ((mShooter.checkIfPersistentlyHasCorrectSpeed(mShooterMode)
            && lowGoalExtensionConditionsMet
            && highGoalDriveByExtensionConditonsMet)
            || mHasShot) 
            && specialTestVariableToEnableShooting
            )
        {
            if (!mHasShot)
            {
                SmartDashboard.putNumber("Distance first sees the value", value);
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