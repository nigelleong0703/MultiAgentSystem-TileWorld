package tileworld.agent;
import sim.util.Bag;
import sim.util.Int2D;
import sim.util.IntBag;
import tileworld.Parameters;
import tileworld.environment.TWDirection;
import tileworld.environment.TWEntity;
import tileworld.environment.TWEnvironment;
import tileworld.environment.TWFuelStation;
import tileworld.environment.TWHole;
import tileworld.environment.TWObject;
import tileworld.environment.TWTile;
import tileworld.exceptions.CellBlockedException;
import tileworld.Parameters;
import tileworld.environment.TWEnvironment;

public class MyAgentlyy extends TWAgent{

    private static final Object TWTile = null;
    private static final Object TWHole = null;
    private String name = "MyAgentlyy";
    private MyMemory memory;


    enum Mode{
        EXPLORE, COLLECT, FILL, REFUEL, WAIT, FIND_FUEL_STATION
    }

    public MyAgentlyy(int xpos, int ypos, TWEnvironment env, double fuelLevel) {
        super(xpos, ypos, env, fuelLevel);
        this.name = name;
        this.memory = new MyMemory(this, env.schedule, env.getxDimension(), env.getyDimension());
        this.sensor = new TWAgentSensor(this, Parameters.defaultSensorRange);
    }

    @Override
    public String getName() {
        return name;
    }

    @Override
    public void communicate(){
        // Create a new message with agent's name
    }

    @Override
    protected TWThought think() {
        //TODO
        sense();

        return null;
    }

    @Override
    protected void act(TWThought thought) {
        //TODO
    }



}
