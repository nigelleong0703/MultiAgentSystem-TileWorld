package tileworld.agent;

import sim.util.Bag;
import sim.util.Int2D;
import tileworld.environment.TWEntity;

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

    // // 想法是如果一个人准备去取一个tile 那他就要通知大家这个我取了，你们可以删除记忆了请各位bye bye，晚安玛卡巴卡
    // public void addRemovedItem(TWEntity o){
    //     this.addMessage("removeItem", o);
    // }

    // public TWEntity getRemovedItem(){
    //     if (this.messages.containsKey("removeItem")) {
    //         return (TWEntity) this.messages.get("removeItem");
    //     }
    //     else return null;
    // }

    // 用于传播自己的targetGoal
    public void addTargetGoal(Bag targetGoal){
        this.addMessage("targetGoal", targetGoal);
    }
    public Bag getTargetGoal(){
        if (this.messages.containsKey("targetGoal")){
            return (Bag) this.messages.get("targetGoal");
        }
        else return null;
    }
    public void addCompletedGoal(Bag completedGoal){
        this.addMessage("completedGoal", completedGoal);
    }
    public Bag getCompletedGoal(){
        if (this.messages.containsKey("completedGoal")){
            return (Bag) this.messages.get("completedGoal");
        }
        else return null;
    }
}
