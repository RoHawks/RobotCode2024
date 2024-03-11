package robotcode.autonomous;

import java.util.ArrayList;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Step 
{
    private ArrayList<AAction> mActions;
    private String mName;

    public String GetName()
    {
        return mName;
    }

    public Step(String pName)
    {
        mName = pName;
        mActions = new ArrayList<AAction>();
    }

    public void AddAction(AAction pAction)
    {
        mActions.add(pAction);
    }

    public boolean Run()
    {
        StringBuilder log = new StringBuilder();
        log.append(mName);
        log.append("(");
        log.append(mActions.size());
        log.append(")");
        log.append(" :");

        boolean allActionsComplete = true;
        for(int i = 0; i < mActions.size(); i++)
        {
            AAction action = mActions.get(i);
            boolean actionIsComplete = action.Run();
            log.append("[");
            log.append(action.getClass().getName());
            log.append("-");
            log.append(actionIsComplete ? "C": "R");
            log.append("");

            log.append("]");
            if(!actionIsComplete)
            {
                allActionsComplete = false;
            }
            
        }
        
        SmartDashboard.putString("AutonomousStatus", log.toString());
        return allActionsComplete;
    }

    public void EnterStep()
    {
        for(int i = 0; i < mActions.size(); i++)
        {
            AAction action = mActions.get(i);
            action.EnterAction();
        }
    }

    public void EndStep()
    {
        for(int i = 0; i < mActions.size(); i++)
        {
            AAction action = mActions.get(i);
            action.EndAction();
        }
    }
}
