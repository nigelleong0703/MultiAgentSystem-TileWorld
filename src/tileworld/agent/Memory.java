package tileworld.agent;

import sim.engine.Schedule;
import sim.util.Bag;
import sim.util.IntBag;
import tileworld.environment.TWEntity;
import tileworld.environment.TWFuelStation;
import tileworld.environment.TWObject;

import java.util.ArrayList;
import java.util.ArrayList;

public class Memory extends TWAgentWorkingMemory{
    private int[] fuelStation = new int[]{-1, -1};
    private ArrayList<int[]> otherAgentPosition = new ArrayList<int[]>();
    private int[] otherAgentCarriedTiles = new int[5];
    

    public Memory(TWAgent moi, Schedule schedule, int x, int y){
        super(moi, schedule, x, y);
        for (int i = 0; i < 5; i++) {
            otherAgentPosition.add(new int[]{-1, -1});
        }
    }
    
    public void setFuelStation(int x, int y) {
        fuelStation = new int[]{x,y};
    }

    public void setFuelStation(TWEntity object){
        if(object instanceof TWFuelStation){
            fuelStation = new int[]{object.getX(), object.getY()};
        }
        else{
            throw new IllegalArgumentException("Object is not a fuel station");
        }
    }

    public int[] getFuelStation() {
        return this.fuelStation;
    }

    public void updateOtherAgentPosition(int agentnumber, int x, int y){
        int[] position = this.otherAgentPosition.get(agentnumber-1);
        position[0] = x;
        position[1] = y;
    }

    public void updateOtherAgentCarriedTiles(int agentnumber, int carriedTiles){
        this.otherAgentCarriedTiles[agentnumber-1] = carriedTiles;
    }

}
