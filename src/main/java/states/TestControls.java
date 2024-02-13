package states;

import edu.wpi.first.wpilibj.XboxController;

public class TestControls implements Controls {

    XboxController mController; 

    public TestControls(XboxController pController)
    {
        mController = pController;
    }

    @Override
    public boolean GetForceIntakingMode() {
        return mController.getLeftBumper();
    }

    @Override
    public boolean GetForceEjectionMode() {
       return mController.getRightBumper();
    }

    @Override
    public boolean GetPrepareForHighGoalManual() {
        return false;
    }

    @Override
    public boolean GetPrepareForHighGoalDriveBy() {
        return mController.getXButton();
    }

    @Override
    public boolean GetPrepareForLowGoal() {
        return mController.getAButton();
    }

    @Override
    public boolean GetPrepareForAutoAim() {
        // return mController.getBButton();
        return false;
    }

    @Override
    public boolean GetStartShootingSequence() {
        return mController.getBButton();
    }

    @Override
    public boolean GetTrackScoringAngle() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'GetTrackScoringAngle'");
    }

    @Override
    public double GetSwerveXComponent() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'GetSwerveXComponent'");
    }

    @Override
    public double GetSwerveYComponent() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'GetSwerveYComponent'");
    }

    @Override
    public double GetSwerveLinearSpeed() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'GetSwerveLinearSpeed'");
    }

    @Override
    public double GetSwerveRotationalSpeed() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'GetSwerveRotationalSpeed'");
    }

    @Override
    public boolean GetSwerveBrakes() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'GetSwerveBrakes'");
    }

    @Override
    public boolean GetSwerveModeSwitch() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'GetSwerveModeSwitch'");
    }

    @Override
    public boolean GetManualControlsMode() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'GetManualControlsMode'");
    }

    @Override
    public universalSwerve.utilities.SwerveNudgingDirection GetSwerveNudgingDirection() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'GetSwerveNudgingDirection'");
    }

    @Override
    public boolean GetManualAimCounterClockwise() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'GetManualAimCounterClockwise'");
    }

    @Override
    public boolean GetManualAimClockwise() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'GetManualAimClockwise'");
    }

    @Override
    public boolean GetReleaseGamePiece() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'GetReleaseGamePiece'");
    }

    @Override
    public void TurnOnVibrate() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'TurnOnVibrate'");
    }

    @Override
    public void TurnOffVibrate() {
        // TODO Auto-generated method stub
        throw new UnsupportedOperationException("Unimplemented method 'TurnOffVibrate'");
    }
    
}
