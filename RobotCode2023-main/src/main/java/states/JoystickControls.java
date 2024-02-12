package states;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
 
public class JoystickControls implements Controls
{
    private XboxController mMainController;
    private XboxController mAlternateController;

    public JoystickControls( XboxController pMainController, XboxController pAlternateController)
    {
        mMainController = pMainController;
        mAlternateController = pAlternateController;
    }

    public boolean GetSwitchAutoOption()
    {
        return mAlternateController.getPOV() == 270;
    }

    public boolean GetSwitchAutoDirection()
    {
        return mAlternateController.getPOV() == 90;//same as top
    }

    public boolean GetForceIntakingMode()
    {
        return mAlternateController.getBButton();
    }

    public boolean GetForceHoldingMode()
    {
        return mAlternateController.getXButton();
    }

    public boolean GetForceEjectionMode()
    {
        return mAlternateController.getYButton();
    }



    public boolean GetPrepareForHigh()
    {
        return mAlternateController.getPOV() == 90;
    }

    public boolean GetPrepareForMiddle()
    {
        return mAlternateController.getPOV() == 0;
    }
    
    public boolean GetPrepareForLow()
    {
        return mAlternateController.getPOV() == 189;
    }

    public GamePieces GetGamePieceMode()
    {
        return mAlternateController.getRightTriggerAxis() > 0.5 ? GamePieces.CUBE : GamePieces.CONE;
    }

    public boolean GetInvokeScoringRoutine()
    {
        return  mMainController.getLeftBumper();
    }
    
    public boolean GetTrackScoringAngle()
    {
        return mAlternateController.getPOV() == 270;
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

}