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
    private int[][] otherAgentPosition = new int[5][2];
    private int[] otherAgentCarriedTiles = new int[5];

    public Agent1(String name, int xpos, int ypos, TWEnvironment env, double fuelLevel) {
        super(name, xpos, ypos, env, fuelLevel);
        this.name = name;
        clearMessage();
        for (int i = 0; i < this.otherAgentPosition.length; i++) {
            // if position is unknown, it is declared as -1
            this.otherAgentPosition[i][0] = -1; 
            this.otherAgentPosition[i][1] = -1; 
        }

    }

    private clearMessage(){
        this.privateMessage = new String[] {"","","","",""};
        this.publicMessage = "";
    }
    
    private addPrivateMessage(int agentnumber, String message){
        // agent number should be 1-5
        this.privateMessage[agentnumber-1] = this.privateMessage[agentnumber] + ";" + message;
    }

    private addPublicMessage(String message){
        this.publicMessage = this.publicMessage + ";" + message;
    }

    public void updateOtherAgentPosition(int agentnumber, int x, int y){
        this.otherAgentPosition[agentnumber-1][0] = x;
        this.otherAgentPosition[agentnumber-1][1] = y;
    }

    public void updateOtherAgentCarriedTiles(int agentnumber, int carriedTiles){
        this.otherAgentCarriedTiles[agentnumber-1] = carriedTiles;
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

    protected TWThought think() {
    //        getMemory().getClosestObjectInSensorRange(Tile.class);
        // System.out.println("Simple Score: " + this.score);
        // return new TWThought(TWAction.MOVE,getRandomDirection());
    }
 
    @Overridemmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmmm
    protected void act(TWThought thought) {
        // get the object based on the memory
        Object currentPositionObject = this.getMemory().getMemoryGrid().get(this.getX(), this.getY());
        switch(thought.getAction()){
        // MOVE, PICKUP, PUTDOWN, REFUEL;    
            case PICKUP:
                if (currentPositionObject instanceof TWTile){
                    pickUpTile((TWTile)currentPositionObject);
                    this.getMemory().removeObject(this.getX(), this.getY());
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
                    this.getMemory().removeObject(this.getX(), this.getY());
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
