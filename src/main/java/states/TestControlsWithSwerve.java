package states;

import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import universalSwerve.utilities.SwerveNudgingDirection;


public class TestControlsWithSwerve implements Controls {

    XboxController mMainController; 
    XboxController mAlternateController; 

    public TestControlsWithSwerve(XboxController pMainController, XboxController pAlternateController)
    {
        mMainController = pMainController;
        mAlternateController = pAlternateController;
    }

    @Override
    public boolean GetForceIntakingMode() {
        return mAlternateController.getLeftBumper();
    }

    @Override
    public boolean GetForceEjectionMode() {
       return mAlternateController.getRightBumper();
    }

    @Override
    public boolean GetPrepareForHighGoalManual() {
        return false;
    }

    @Override
    public boolean GetPrepareForHighGoalDriveBy() {
        SmartDashboard.putBoolean("High Goal Drive", mAlternateController.getXButton());
        return mAlternateController.getXButton();
    }

    @Override
    public boolean GetPrepareForLowGoal() {
        return mAlternateController.getAButton();
    }

    @Override
    public boolean GetPrepareForAutoAim() {
        // return mController.getBButton();
        return false;
    }

    @Override
    public boolean GetStartShootingSequence() {
        // return mMainController.getLeftBumper() && mMainController.getRightBumper();
        return mAlternateController.getBButton();
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
        return triggerValue * triggerValue;
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

        if(mMainController.getAButton())
        {
            return SwerveNudgingDirection.SOUTH;
        }
        else if(mMainController.getXButton())
        {
            return SwerveNudgingDirection.EAST;
        }
        else if(mMainController.getYButton())
        {
            return SwerveNudgingDirection.NORTH;
        }
        else if(mMainController.getBButton())
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
        return mAlternateController.getLeftTriggerAxis() > 0.5;
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


    
}
