package tileworld.agent;

import sim.display.Console;
import tileworld.TWGUI;
import tileworld.environment.TWDirection;
import tileworld.environment.TWEntity;
import tileworld.environment.TWEnvironment;
import tileworld.environment.TWHole;
import tileworld.environment.TWTile;
import tileworld.environment.TWFuelStation;
import tileworld.exceptions.CellBlockedException;
import tileworld.planners.AstarPathGenerator;
import tileworld.planners.TWPath;
import tileworld.planners.TWPathStep;
import tileworld.environment.TWObstacle;

import java.util.Arrays;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;


public class Agent1 extends TWAgent {
    private String name="Agent1";
    private String[] privateMessage;
    private String publicMessage;
    private final int mapsizeX = this.getEnvironment().getxDimension();
    private final int mapsizeY = this.getEnvironment().getyDimension();
    private int[][] observedMap = new int[mapsizeX][mapsizeY];
    private AstarPathGenerator pathGenerator = new AstarPathGenerator(this.getEnvironment(), this, mapsizeX+mapsizeY);


    public Agent1(String name, int xpos, int ypos, TWEnvironment env, double fuelLevel) {
        super(xpos, ypos, env, fuelLevel);
        this.name = name;
        clearMessage();
    }

    private void clearMessage(){
        this.privateMessage = new String[] {"","","","",""};
        this.publicMessage = "";
    }
    
    private void addPrivateMessage(int agentnumber, String message){
        // agent number should be 1-5
        this.privateMessage[agentnumber-1] = this.privateMessage[agentnumber] + ";" + message;
    }

    private void addPublicMessage(String message){
        this.publicMessage = this.publicMessage + ";" + message;
    }

    @Override
    public void communicate() {
        // Message message = new Message("","","");
        // this.getEnvironment().receiveMessage(message); // this will send the message to the broadcast channel of the environment
        
        // message(from,to, message)
        if (!this.publicMessage.equals("")){
            this.getEnvironment().receiveMessage(new Message(this.name, "all", this.publicMessage));
        }
        for (int i = 0; i < 5; i++){
            if (!this.privateMessage[i].equals("")){
                this.getEnvironment().receiveMessage(new Message(this.name, "Agent"+(i+1), this.privateMessage[i]));
            }
        }
        this.clearMessage();
    }

    private void ReceiveMessage(){
        // message(from,to, message)
        ArrayList<Message> receivedMessage = this.getEnvironment().getMessages();
        for (Message message: receivedMessage){
            if (this.name.equals(message.getTo())){
                System.out.println(this.name + " Received message from " + message.getFrom() + " to " +message.getTo() + ": " + message.getMessage()); 

                // As the message contains several event, which is split by ";"
                String[] messageSplit = message.getMessage().split(";");
                
                for (String mes: messageSplit){
                    String[] tempMes = mes.split(" ");
                    String messageTopic = tempMes[0];
                    switch(messageTopic){
                        case "Request":
                            continue;
                        case "Require":
                            continue;
                        case "GoFindFuelStation":
                            continue;
                    }

                }
            }
            if (message.getTo().equals("all")){
                String[] messageSplit = message.getMessage().split(";");
                for (String mes: messageSplit){
                    String[] tempMes = mes.split(" ");
                    String messageTopic = tempMes[0];
                    switch(messageTopic){
                        case "FindFuelStation":
                            continue;
                        case "UpdateMemoryMap":
                            continue;
                        case "AgentPosition":
                            continue;
                        case "AgentCarriedTiles":
                            continue;
                    }
                }
            }
                
        }
    }

    protected TWThought think() {
    //        getMemory().getClosestObjectInSensorRange(Tile.class);
        // System.out.println("Simple Score: " + this.score);
        // return new TWThought(TWAction.MOVE,getRandomDirection());
        this.ReceiveMessage();
        return new TWThought(TWAction.MOVE,getRandomDirection());
    }

 
    @Override
    protected void act(TWThought thought) {
        // get the object based on the memory
        Object currentPositionObject = this.getMemory().getMemoryGrid().get(this.getX(), this.getY());
        switch(thought.getAction()){
        // MOVE, PICKUP, PUTDOWN, REFUEL;    
            case PICKUP:
                if (currentPositionObject instanceof TWTile){
                    pickUpTile((TWTile)currentPositionObject);
                    // TWAgent twentity = new TWEntity(this.getX(), this.getY(), this.getEnvironment());
                    TWEntity e = (TWEntity) this.getEnvironment().getObjectGrid().get(this.getX(), this.getY());
                    this.getMemory().removeObject(e);
                    this.addPublicMessage("UpdateMemoryMap " + this.getX() + " " + this.getY()+" " + "null");
                    act(this.think());
                }
                else{
                    System.out.println("No tile to pick up");
                }
                return;

            case PUTDOWN:
                if (currentPositionObject instanceof TWHole){
                    putTileInHole((TWHole)currentPositionObject);
                    TWEntity e = (TWEntity) this.getEnvironment().getObjectGrid().get(this.getX(), this.getY());
                    this.getMemory().removeObject(e);
                    this.addPublicMessage("UpdateMemoryMap " + this.getX() + " " + this.getY()+" " + "null");
                    act(this.think());
                }
                else{
                    System.out.println("No hole to put tile in");
                }
                return;

            case REFUEL:
                refuel();
                act(this.think());
                return;
                

        }
        try {
            this.move(thought.getDirection());
            this.addPublicMessage("AgentPosition " + this.name + " " + this.getX() + " " + this.getY()); 
            this.addPublicMessage("AgentCarriedTiles " + this.name + " " + this.carriedTiles.size());
        } catch (CellBlockedException ex) {

            // Cell is blocked, replan?
            act(this.think());
        }
    }
            
    private TWDirection getRandomDirection(){

        TWDirection randomDir = TWDirection.values()[this.getEnvironment().random.nextInt(5)];

        if(this.getX()>=this.getEnvironment().getxDimension() ){
            randomDir = TWDirection.W;
        }else if(this.getX()<=1 ){
            randomDir = TWDirection.E;
        }else if(this.getY()<=1 ){
            randomDir = TWDirection.S;
        }else if(this.getY()>=this.getEnvironment().getxDimension() ){
            randomDir = TWDirection.N;
        }

       return randomDir;

    }

    @Override
    public String getName() {
        return name;
    }
}
