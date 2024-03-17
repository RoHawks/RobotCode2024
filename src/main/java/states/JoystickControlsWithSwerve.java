package states;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import universalSwerve.utilities.SwerveNudgingDirection;


public class JoystickControlsWithSwerve implements Controls {

    XboxController mMainController; 
    XboxController mAlternateController; 

    public JoystickControlsWithSwerve(XboxController pMainController, XboxController pAlternateController)
    {
        mMainController = pMainController;
        mAlternateController = pAlternateController;
    }


    public boolean GetForceIntakingMode() {
        return mAlternateController.getLeftBumper();
    }

    @Override
    public boolean GetForceEjectionMode() {
       return mAlternateController.getYButton();
    }

    @Override
    public boolean GetPrepareForHighGoalManual() {
        return mMainController.getBButton();
     
    }

    @Override
    public boolean GetPrepareForHighGoalDriveBy() {
        //SmartDashboard.putBoolean("High Goal Drive", mAlternateController.getXButton());
        return mMainController.getXButton();
    }

    @Override
    public boolean GetPrepareForLowGoal() {
        return mMainController.getAButton();
    }

    @Override
    public boolean GetPrepareForAutoAim() {
        return mMainController.getYButton();

    }

    @Override
    public boolean GetStartShootingSequence() {
        return mMainController.getLeftBumper() && mMainController.getRightBumper();
        // return mMainController.getBButton();
    }

    
    public boolean GetPrepareToClimb() {
        return mAlternateController.getYButton();
    }

    public boolean GetRetractClimb()
    {
        return mAlternateController.getXButton();
    }


    public double GetSwerveXComponent()
    {
        return mMainController.getLeftX();
    }

    public double GetSwerveYComponent()
    {
        return mMainController.getLeftY();
    }

    public double GetSwerveLinearSpeed()
    {
        //square to allow for easy speed control on low speeds
        double triggerValue = mMainController.getRightTriggerAxis();
        return 1.0 * triggerValue * triggerValue;
    }

    public double GetSwerveRotationalSpeed()
    {
        //Let's try squaring this for finer grained slowcontrol
        double rawVal = mMainController.getRightX();
        return rawVal > 0 ? rawVal * rawVal : -1.0 * rawVal * rawVal; 
    }


    public boolean GetSwerveModeSwitch()
    {
        return mMainController.getStartButton();
    }

    public SwerveNudgingDirection GetSwerveNudgingDirection()
    {
        
        int angle = mMainController.getPOV();

        SmartDashboard.putNumber("D Pad Angle", angle);
        if(angle == 180|| angle == 225)
        {
            return SwerveNudgingDirection.SOUTH;
        }
        else if(angle == 270 || angle == 315)
        {
            return SwerveNudgingDirection.EAST;
        }
        else if(angle == 0 || angle == 45)
        {
            return SwerveNudgingDirection.NORTH;
        }
        else if(angle == 90 || angle == 135)
        {
            return SwerveNudgingDirection.WEST;
        }
        else
        {
            return SwerveNudgingDirection.NONE;
        }
    }


    @Override
    public boolean GetSwerveBrakes() {
        return false; //GIVE ACTUAL LATER
    }

    public boolean GetManualControlsMode()
    {
        //SmartDashboard.putNumber("InJBC_LTR", mAlternateController.getLeftTriggerAxis());        
        return false;
    }

    public boolean GetManualAimCounterClockwise()
    {
        return mAlternateController.getRightBumper();
    }
    public boolean GetManualAimClockwise()
    {
        return mAlternateController.getStartButton();
        
    }

    public boolean GetReleaseGamePiece()
    {
        return mMainController.getRightBumper();
    }

    public boolean GetIsLowGoalMode()
    {
        return mAlternateController.getLeftY() < -0.5;
    }

    public void TurnOnVibrate()
    {
        mMainController.setRumble(RumbleType.kBothRumble, 1);
    }
    public void TurnOffVibrate()
    {
        mMainController.setRumble(RumbleType.kBothRumble, 0);
    }

    @Override
    public boolean GetTrackSpecificAngle() {
       return false;
    }

    @Override
    public double GetSpecificAngleToTrack() {
        return -1;
    }

    public boolean GetAutonomousModeSelection()
    {
        return mAlternateController.getRightTriggerAxis() > 0.75;
    }

    public boolean GetLoggingEnabledToggle()
    {
        return false;
        //return mAlternateController.getLeftY() < 0.25;
    }   
    
    public boolean TestOnly_AnglerUp()
    {
        return mAlternateController.getRightBumper();
    }

    public boolean TestOnly_AnglerDown()
    {
        return mAlternateController.getPOV() == 90;
    }

    public boolean TestOnly_TopConveyorOn()
    {
        return mAlternateController.getYButton();
    }

    public boolean TestOnly_BottomConveyorOn()
    {
        return mAlternateController.getXButton();
    }

    public boolean TestOnly_IntakeRollerOn()
    {
        return mAlternateController.getPOV() == 0;
    }

    public boolean TestOnly_TopShooterOn()
    {
        return mAlternateController.getPOV() == 180;
    }

    public boolean TestOnly_BottomShooterOn()
    {
        return mAlternateController.getLeftBumper();
    }

    public boolean TestOnly_WestArmUp()
    {
        return mAlternateController.getAButton();
    }

    public boolean TestOnly_WestArmDown()
    {
        return mAlternateController.getPOV() == 270;
    }

    public boolean TestOnly_EastArmUp()
    {
        return mAlternateController.getBButton();
    }

    public boolean TestOnly_EastArmDown()
    {
        return mAlternateController.getLeftY() > 0.75;
    }

    public boolean TestOnly_ExtendoArmOut()
    {
        return mAlternateController.getRightStickButton();
    }

    public boolean TestOnly_ExtendoArmIn()
    {
        return mAlternateController.getLeftStickButton();
    }

    public boolean TestOnly_TrapMechanismOn()
    {
        return mAlternateController.getStartButton();
    }

    public boolean TestOnly_AllowSwerveDuringTestMode()
    {
        return mAlternateController.getBackButton();
    }

}
