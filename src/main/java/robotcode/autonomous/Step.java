package robotcode.autonomous;

import java.util.ArrayList;

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
        boolean allActionsComplete = true;
        for(int i = 0; i < mActions.size(); i++)
        {
            AAction action = mActions.get(i);
            boolean actionIsComplete = action.Run();
            if(!actionIsComplete)
            {
                allActionsComplete = false;
            }
        }

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
