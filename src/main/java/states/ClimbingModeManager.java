package states;

public class ClimbingModeManager {
    public static enum ClimbingMode
    {
        Off,
        Extending,
        Retracting
    }

    private static ClimbingMode currentClimbingMode = ClimbingMode.Off;

    public static ClimbingMode getClimbingMode()
    {
        return currentClimbingMode;
    }

    public static void setClimbingMode(ClimbingMode pClimbingMode)
    {
        currentClimbingMode = pClimbingMode;
    }

    public static void determineClimbingMode(Controls pControls)
    {
        if (pControls.GetPrepareToClimb())
        {
            ClimbingModeManager.setClimbingMode(ClimbingMode.Extending);
        }
        else if (pControls.GetRetractClimb() && ClimbingModeManager.getClimbingMode() == ClimbingMode.Extending)
        {
            ClimbingModeManager.setClimbingMode(ClimbingMode.Retracting);
        }

    }
}
