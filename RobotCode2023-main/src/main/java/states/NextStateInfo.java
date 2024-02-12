package states;

public class NextStateInfo
{
    private States mNextState;
    private Object mNextStateParameter;

    public NextStateInfo(States pNextState)
    {
        mNextState = pNextState;
        mNextStateParameter = null;
    }
    
    public NextStateInfo(States pNextState, Object pNextStateParameter)
    {
        mNextState = pNextState;
        mNextStateParameter = pNextStateParameter;
    }

    public States GetNextState()
    {
        return mNextState;
    }

    public Object GetNextStateParameter()
    {
        return mNextStateParameter;
    }
}
