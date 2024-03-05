package states;

import universalSwerve.controls.ISwerveControls;
import universalSwerve.utilities.SwerveNudgingDirection;

public interface Controls extends ISwerveControls
{
    public boolean GetForceIntakingMode();
    public boolean GetForceEjectionMode();

    public boolean GetPrepareForHighGoalManual();
    public boolean GetPrepareForHighGoalDriveBy(); 
    public boolean GetPrepareForLowGoal();
    public boolean GetPrepareForAutoAim();

    public boolean GetStartShootingSequence();


    
    public double GetSwerveXComponent();
    public double GetSwerveYComponent();
    public double GetSwerveLinearSpeed();
    public double GetSwerveRotationalSpeed();
    public boolean GetSwerveBrakes();

    public boolean GetSwerveModeSwitch();

    public boolean GetManualControlsMode();

    public universalSwerve.utilities.SwerveNudgingDirection GetSwerveNudgingDirection();

    public boolean GetManualAimCounterClockwise();
    public boolean GetManualAimClockwise();

    public boolean GetPrepareToClimb();
    public boolean GetRetractClimb();

    public boolean GetReleaseGamePiece();

    public void TurnOnVibrate();
    public void TurnOffVibrate();


}