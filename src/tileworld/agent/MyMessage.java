package tileworld.agent;

import sim.util.Bag;
import sim.util.Int2D;

import java.util.Collection;
import java.util.HashMap;
import java.util.Map;
import java.util.Set;

public class MyMessage extends Message{
    private Int2D fuelStation;
    private Bag sensedObjects;
    private Int2D posAgent;
    private HashMap<String, Object> messages;

    //	Object objects [];
    public MyMessage(String from, String to){
        super(from, to, null);
        this.messages = new HashMap<String, Object>();
    }

    public MyMessage(String from, String to, HashMap<String, Object> messages) {
        super(from, to, messages.toString());
        this.messages = messages;
    }

    public HashMap<String, Object> getMessages(){
        return this.messages;
    }

    public void addMessage(String key, Object value) {
        this.messages.put(key, value);
    }

    public void addFuelStationPosition(Int2D fuelStationPosition) {
        if(fuelStationPosition != null){
            System.out.println("Receive fuel station position in message");
        }
        this.addMessage("fuelStationPosition", fuelStationPosition);
    }

    public Int2D getFuelStationPosition() {
        if (this.messages.containsKey("fuelStationPosition")) {
            return (Int2D) this.messages.get("fuelStationPosition");
        }
        else return null;
    }

    public void addSensedObjects(Bag sensedObjects, Int2D agentPosition) {
        this.addMessage("sensedObjects", sensedObjects);
        this.addMessage("agentPosition", agentPosition);
    }

    public Bag getSensedObjects() {
        if (this.messages.containsKey("sensedObjects")) {
            return (Bag) this.messages.get("sensedObjects");
        }
        else return null;
    }

    public Int2D getAgentPosition() {
        if (this.messages.containsKey("agentPosition")) {
            return (Int2D) this.messages.get("agentPosition");
        }
        else return null;
    }


}
