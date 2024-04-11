package tileworld.agent;

import sim.display.Console;
import sim.portrayal.Inspector;
import sim.portrayal.LocationWrapper;
import sim.portrayal.Portrayal;
import sim.util.Bag;
import sim.util.Int2D;
import sim.util.IntBag;
import tileworld.Parameters;
import tileworld.TWGUI;
import tileworld.environment.TWDirection;
import tileworld.environment.TWEntity;
import tileworld.environment.TWEnvironment;
import tileworld.environment.TWHole;
import tileworld.environment.TWObject;
import tileworld.environment.TWTile;
import tileworld.environment.TWFuelStation;
import tileworld.exceptions.CellBlockedException;
import tileworld.planners.TWPath;
import tileworld.planners.TWPathStep;
import tileworld.planners.TWPlannerLZH;
import tileworld.environment.TWObstacle;
import tileworld.agent.MyMemory;
import tileworld.agent.MyMessage;
import tileworld.agent.TWAgentSensor;
import tileworld.agent.TWAgent;

import java.util.Arrays;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.PriorityQueue;
import java.util.List;

import java.util.Scanner;

public class AgentLZH extends TWAgent {
    private String name;
    private int index;
    private final int mapsizeX = this.getEnvironment().getxDimension();
    private final int mapsizeY = this.getEnvironment().getyDimension();

    enum Mode {
        EXPLORE, COLLECT, FILL, REFUEL, WAIT, FIND_FUELSTATION
    }
    
    private Mode mode;
    private double fuelThreshold;
    private TWPlannerLZH planner;
    private MyMemory memory;
    // private TWAgentSensor sensor;

    public AgentLZH(int index, String name, int xpos, int ypos, TWEnvironment env, double fuelLevel) {
        super(xpos, ypos, env, fuelLevel);
        this.index = index;
        this.name = name;
        // fuel threshold
        this.fuelThreshold = Math.max(Parameters.xDimension, Parameters.yDimension) * 1.2;
        // Sensor
        this.sensor = new TWAgentSensor(this, Parameters.defaultSensorRange);
        // 路径规划
        this.planner = new TWPlannerLZH(this);
        // Memory
        this.memory = new MyMemory(this, env.schedule, mapsizeX, mapsizeY);

    }
    public AgentLZH(int xpos, int ypos, TWEnvironment env, double fuelLevel) {
        super(xpos, ypos, env, fuelLevel);
        // this.planner = new AstarPathGenerator(this.getEnvironment(), this, mapsizeX+mapsizeY);
        this.planner = new TWPlannerLZH(this);
    }

    @Override
    public void communicate() {
        MyMessage message = new MyMessage(this.getName(), "");
        Bag sensedObjects = new Bag();
        IntBag objectXCoords = new IntBag();
        IntBag objectYCoords = new IntBag();

        // 从sensedobject中更新memory
        this.memory.getMemoryGrid().getNeighborsMaxDistance(x, y, Parameters.defaultSensorRange, false, sensedObjects, objectXCoords, objectYCoords);
        // System.out.print
        
        // 发送消息位置
        // 油站位置
        System.out.println(this.memory.getFuelStation());
        message.addFuelStationPosition(this.memory.getFuelStation());
        // sensedObjects位置和自身位置
        message.addSensedObjects(sensedObjects, new Int2D(x, y));
        System.out.println(message);
        // 发到environment
        this.getEnvironment().receiveMessage(message);
    }

    private ArrayList<TWEntity> detectObjectNoCollision(PriorityQueue<TWEntity> neighborObjects, List<TWAgent> neighbouringAgents) {
        // TODO Auto-generated method stub
        ArrayList<TWEntity> tiles = new ArrayList<TWEntity>();
        // while(!neighborObjects.isEmpty()) {
        //     TWEntity tile = neighborObjects.poll();
        //     for (int i = 0; i < neighbouringAgents.size(); i++) {
        //         // 有其他agent离tile更近, 这个agent不是inspector
        //         if (!(neighbouringAgents.get(i) instanceof MyInspectorAgent)
        //                 && this.getDistanceTo(tile)/3 >= neighbouringAgents.get(i).getDistanceTo(tile)) {
        //             continue;
        //         }
        //         tiles.add(tile);
        //     }
        // }
        for(TWEntity object: neighborObjects) {
            boolean collision = false;
            for (TWAgent agent : neighbouringAgents) {
                // 如果agent离object更近
                //
                // 为什么是/3? 
                if (agent.getDistanceTo(object) <= this.getDistanceTo(object)/3) {
                    collision = true;
                    break;
                }
            }
            if (!collision) {
                tiles.add(object);
            }
        }
        return (tiles.size() > 0) ? tiles: null;
    }


    private Int2D getPositionAdd(Int2D base, Int2D position) {
        return new Int2D (base.x + position.x, base.y + position.y) ;
    }

    // 这个逻辑不是很对，需要改，这个是分层之后往回走来走去扫描

    public void addGoalsForFuelStation() {
        planner.voidGoals();
        planner.voidPlan();
        Int2D [] basePos = {
            new Int2D(0,0),
            new Int2D(0,Parameters.yDimension/5),
            new Int2D(0,(Parameters.yDimension/5) * 2),
            new Int2D(0,(Parameters.yDimension/5) * 3),
            new Int2D(0,(Parameters.yDimension/5) * 4),
        };
        Int2D base = basePos[this.index];
        Int2D position = new Int2D(Parameters.defaultSensorRange, Parameters.defaultSensorRange);
        int depth= Parameters.defaultSensorRange;

        while(depth < Parameters.yDimension/5){
            int posX = position.x;
            int posY = position.y;
            planner.getGoals().add(getPositionAdd(base, position));

            // point1 位于当前基地的右侧， 与当前基地的距离为Paramaeters.defaultSensorRange, 与环境的右边缘的距离为Parameters.defaultSensorRange
            posX += Parameters.xDimension - Parameters.defaultSensorRange-1;
            position = new Int2D(posX,position.y);
            planner.getGoals().add(getPositionAdd(base, position));

            // agent 的深度， 从最顶部下来的深度(中心位置)
            depth = depth + Parameters.defaultSensorRange * 2 + 1;
            // 在最后一行，如果往下走的距离会超过strip，则让中心刚刚好达到最底下的边界
            if(depth >= Parameters.yDimension/5){
                //
                posY= Parameters.yDimension/5 - 1;
                depth = Parameters.yDimension/5 - 1;
                position = new Int2D(position.x, posY);
                planner.getGoals().add(getPositionAdd(base, position));

                //
                posX = Parameters.defaultSensorRange;
                position = new Int2D(posX,position.y);
                planner.getGoals().add(getPositionAdd(base, position));
                break;
            }

            // Point 2
            posY+= Parameters.defaultSensorRange * 2 + 1;
            position = new Int2D(position.x, posY);
            planner.getGoals().add(getPositionAdd(base, position));
            
            //point 3 
            posX = Parameters.defaultSensorRange;
            position = new Int2D(posX, position.y);
            planner.getGoals().add(getPositionAdd(base, position));

            depth += Parameters.defaultSensorRange * 2 + 1;

            //往下走一层，如果也是超过边界，则将中心刚刚好达到最底下的边界
            if(depth > Parameters.yDimension/5){
                posY = Parameters.yDimension/5 - 1;
                posX = Parameters.defaultSensorRange;
                depth = Parameters.yDimension/5 - 1;
                position = new Int2D(posX, posY);
                planner.getGoals().add(getPositionAdd(base, position));
                break;
            }
            
            posY += Parameters.defaultSensorRange * 2 +1;
            position = new Int2D(position.x, posY);
            planner.getGoals().add(getPositionAdd(base, position));
        }
    }


    // 这个也需要进行改进，这个是随机走
    private TWThought RandomMoveThought() {
        ArrayList<TWDirection> dirs = new ArrayList<TWDirection>();
        int x = this.getX();
        int y = this.getY();

        if ((y-1) >= 0
                && !this.memory.isCellBlocked(x, y-1)) {
            dirs.add(TWDirection.N);
        }
        if ((y+1) < this.getEnvironment().getyDimension()
                && !this.memory.isCellBlocked(x, y+1)) {
            dirs.add(TWDirection.S);
        }
        if ((x+1) < this.getEnvironment().getxDimension()
                && !this.memory.isCellBlocked(x+1, y)) {
            dirs.add(TWDirection.E);
        }
        if ((x-1) >= 0
                && !this.memory.isCellBlocked(x-1, y)) {
            dirs.add(TWDirection.W);
        }

        if (dirs.size() > 0) {
            int random_num = this.getEnvironment().random.nextInt(dirs.size());
            return new TWThought(TWAction.MOVE, dirs.get(random_num));
        }
        else {
            System.out.println("No where to go!");
            return new TWThought(TWAction.MOVE, TWDirection.Z);
        }
    }

    private Int2D generateRandomNearCell(Int2D goalPos) {
        // TODO Auto-generated method stub
        ArrayList<Int2D> dirs = new ArrayList<Int2D>();
        int x = goalPos.getX();
        int y = goalPos.getY();

        if ((y-1) >= 0
                && !this.memory.isCellBlocked(x, y-1)) {
            dirs.add(new Int2D(x, y-1));
        }
        if ((y+1) < this.getEnvironment().getyDimension()
                && !this.memory.isCellBlocked(x, y+1)) {
            dirs.add(new Int2D(x, y+1));
        }
        if ((x+1) < this.getEnvironment().getxDimension()
                && !this.memory.isCellBlocked(x+1, y)) {
            dirs.add(new Int2D(x+1, y));
        }
        if ((x-1) >= 0
                && !this.memory.isCellBlocked(x-1, y)) {
            dirs.add(new Int2D(x-1, y));
        }

        if (dirs.size() > 0) {
            int random_num = this.getEnvironment().random.nextInt(dirs.size());
            return dirs.get(random_num);
        }
        else {
            System.out.println("No where to go!");
            return null;
        }
    }
    
    @Override
    protected TWThought think() {
        /** 
         * get message and update memory
         */

        
//         ArrayList<Message> messages = this.getEnvironment().getMessages();
//          for (Message m : messages) {
//              if (m == null || m.getMessage() == null || m.getFrom() == this.getName()) continue;
//              Int2D fuelStationPos = ((MyMessage)m).getFuelStationPosition();
//              if (this.memory.getFuelStation() == null && fuelStationPos != null) {
//                  this.memory.setFuelStation(fuelStationPos.x, fuelStationPos.y);
//  //  				break;
//              }
//              Bag sharedObjects = ((MyMessage)m).getSensedObjects();
//              Int2D posAgent = ((MyMessage)m).getAgentPosition();
//              if (sharedObjects != null) {
//                  this.memory.mergeMemory(sharedObjects, posAgent);
//              }
//          }
        ArrayList<Message> received_message = this.getEnvironment().getMessages();
        for (Message message : received_message) {
            // System.out.println(message);
            MyMessage myMessage = (MyMessage) message;
            Int2D fuelStation = myMessage.getFuelStationPosition();
            // System.out.println(this.name + "receive msg");
            // System.out.println(this.name + " receive msg " + fuelStation + " from " + message.getFrom());
            
            //从消息获得别人sensor object位置，感知物体包括位置和种类
            Bag sharedObjects = myMessage.getSensedObjects();
            //从消息获得agent位置
            Int2D agentPosition = myMessage.getAgentPosition();

            // System.out.println("Check if got message");
            // System.out.println(message == null);
            // // System.out.println(myMessage.getMessage() == null);
            // System.out.println(message.getFrom().equals(this.getName()));
            // System.out.println();
            // 如果没有消息或者发送者是自己，跳过
            if (message == null || message.getFrom().equals(this.getName())) {
                continue;
            }
            
            // 如果自身memory没有fuelstation位置和消息中有fuelstation位置，更新memory
            // System.out.println(this.memory.getFuelStation());
            // System.out.println(this.memory.getFuelStation());
            // System.out.println(fuelStation);

            if(this.memory.getFuelStation()== null && fuelStation != null) {
                System.out.println(this.name + "updating fuel station memory");
                this.memory.setFuelStation(fuelStation.x, fuelStation.y);
                System.out.println(this.memory.getFuelStation());
            }

            // 如果当前消息包含感知物体，则将感知物体和Agent合并一起更新到memory
            if (sharedObjects != null) {
                this.memory.mergeMemory(sharedObjects, agentPosition);
            }
        }

        // 根据agent内存中获取附近的对象，并使用优先级队列按照它们与agent的距离进行排序
        PriorityQueue<TWEntity> TilesList;
        PriorityQueue<TWEntity> HolesList;
        // PriorityQueue<TWEntity> FuelStation;
        // 利用欧几里得距离排序对象与agent的距离

        TilesList = this.memory.getNearbyAllSortedObjects(x, y, 50, TWTile.class);
        HolesList = this.memory.getNearbyAllSortedObjects(x, y, 50, TWHole.class);
        TWEntity FuelStation = this.memory.getNearbyObject(x,y,200,TWFuelStation.class);
        System.out.println(this.name);
        System.out.println(FuelStation);
        System.out.println(memory.getFuelStation());
        System.out.println(TilesList);
        System.out.println(HolesList);
        if(FuelStation != null){
            System.out.println("Fuel Station Found"); 
            Scanner scanner = new Scanner(System.in);
            System.out.println("Press any key to continue...");
            // Wait for user input
            scanner.nextLine();

            System.out.println("Resuming execution.");
            scanner.close();
        }
        // TWObject getNearbyObject(int sx, int sy, double threshold, Class<?> type) 

        TWTile targettile = null;
        TWHole targethole = null;
        // detect whether collide with other agents or not
        ArrayList<TWEntity> tiles = detectObjectNoCollision(TilesList, this.memory.neighbouringAgents);
        ArrayList<TWEntity> holes = detectObjectNoCollision(HolesList, this.memory.neighbouringAgents);
        
        if (tiles != null){
            targettile = (TWTile) tiles.get(0);
        }
        if (holes != null){
            targethole = (TWHole) holes.get(0);
        }
        
        // 为什么写进去同一个位置？？？？
        targettile = this.getMemory().getNearbyTile(x, y, 50);
        targethole = this.getMemory().getNearbyHole(x, y, 50);

        /*Decide the mode based on the current info of agent
         * 
         * Default: EXPLORE
         * found fuel station and fuel is low : REFUEL
         * fuel is below a defind level: WAIT
         * no nearby tiles or holes： EXPLORE
         * has tile on hand: FILL
         * no tile on hand: COLLECT
         */

        // TO DO:
        mode = Mode.EXPLORE;
        if (memory.getFuelStation() == null) {
            mode = Mode.FIND_FUELSTATION;
            System.out.println("Setting mode to find fuel station");
        }
        // if (memory.getFuelStation() == null && (this.getFuelLevel() < this.fuelThreshold)) mode = Mode.FIND_FUELSTATION;
        else if ((memory.getFuelStation() != null) && (this.getFuelLevel() < this.fuelThreshold || this.getFuelLevel() < this.getDistanceTo(memory.getFuelStation().getX(), memory.getFuelStation().getY()))){
            mode = Mode.REFUEL;
        }
        else if (this.getFuelLevel()<this.fuelThreshold){
            mode = Mode.WAIT;
            return new TWThought(TWAction.MOVE, TWDirection.Z);
        }
        else if (mode != Mode.FIND_FUELSTATION){
        // 如果不需要找加油站和不需要加油， 就只剩下EXPLORE, COLLECT和FILL
            if (this.hasTile() && this.carriedTiles.size() < 3 && targethole != null) {
                if (targettile != null && (this.getDistanceTo(targethole.getX(),targethole.getY()) <= this.getDistanceTo(targettile.getX(),targettile.getY()))) {
                    mode = Mode.FILL;
                } else if (targettile != null && (this.getDistanceTo(targethole.getX(),targethole.getY()) > this.getDistanceTo(targettile.getX(),targettile.getY()))){
                    mode = Mode.COLLECT;
                } else if (targettile == null) {
                    mode = Mode.FILL;
                }
            } else if (this.hasTile() && this.carriedTiles.size() < 3 && targethole == null) {
                if (targettile != null) {
                    mode = Mode.COLLECT;
                } else mode = Mode.EXPLORE;
            } else if (this.hasTile() && this.carriedTiles.size() == 3){
                if (targethole != null) {
                    mode = Mode.FILL;
                } else mode = Mode.EXPLORE;
            } else if (!this.hasTile() && targettile != null) {
                mode = Mode.COLLECT;
            } else mode = Mode.EXPLORE;
        } else mode = Mode.EXPLORE;

        System.out.println(mode);
        Object curLocObject = this.memory.getMemoryGrid().get(x, y);
        // 如果不是空地， 那就直接打印出来是什么
        if (curLocObject != null){
            System.out.println("Current Location Obj: " + curLocObject.getClass().getName());
            
        }
        // 如果刚好经过加油站，而且剩下的有少过max的75%，就加油
        if (curLocObject instanceof TWFuelStation && (this.getFuelLevel() < (0.75 * Parameters.defaultFuelLevel))){
            // System.out.println("Current Location is Fuel Station");
            System.out.println("Now Adding Fuel");
            return new TWThought(TWAction.REFUEL, null);
        }
        else if (curLocObject instanceof TWHole && this.getEnvironment().canPutdownTile((TWHole)curLocObject,this) && this.hasTile()){
            System.out.println("Filling Hole");
            return new TWThought(TWAction.PUTDOWN, null);
        }
        else if (curLocObject instanceof TWTile && this.getEnvironment().canPickupTile((TWTile)curLocObject, this) && this.carriedTiles.size()<3){
            System.out.println("Picking up Tile");
            return new TWThought(TWAction.PICKUP, null);
        }

            // 由当前mode 和Goal 确定 Thought
        if (mode == Mode.FIND_FUELSTATION){
            // 如果已经做好了path to fuel station就不需要重新路径规划
            if(this.planner.getGoals().isEmpty()){
                addGoalsForFuelStation();
            }else{ //如果找到了fuelstation就移走这个目标以免重复
                if (this.planner.getGoals().contains(new Int2D(this.x, this.y))) {
                    int index = planner.getGoals().indexOf(new Int2D(this.x, this.y));
                    if (index != -1){ //如果找到了这个目标

                        ////这个需要debug一下
                        System.out.println(index);
                        planner.getGoals().remove(0);
                    }
                }
            }

            for (int i = 0; i<planner.getGoals().size();i++){
                System.out.println("Goals " + i + ": " + planner.getGoals().get(i));
            }
        }else{
            planner.voidGoals();
            planner.voidPlan();

            if (mode == Mode.REFUEL){
                planner.getGoals().add(memory.getFuelStation());
            }
            else if(mode == Mode.FILL){
                planner.getGoals().add(new Int2D(targethole.getX(), targethole.getY()));
            }
            else if (mode == Mode.WAIT){
                return new TWThought(TWAction.MOVE, TWDirection.Z);
            }
            else if (mode == Mode.COLLECT){
                planner.getGoals().add(new Int2D(targettile.getX(), targettile.getY()));
            }
            else if (mode == Mode.EXPLORE){
                return RandomMoveThought();
            }
        }
        
        // for (int i = 0; i < planner.getGoals().size(); i++) {
//			System.out.println("Goals " + i + ": " + planner.getGoals().get(i));
        // }
        if (this.planner.getGoals().isEmpty()){
            return RandomMoveThought();
        }
        //如果计划不为空，则调用plannerzyh.generatePlan()方法生成一条路径计划
        this.planner.generatePlan();

        
        if (!planner.hasPlan()) {
            // 如果没有plan但是需要寻找fuelstation
            if (this.mode == Mode.FIND_FUELSTATION) {
                Int2D newGoal = generateRandomNearCell(planner.getGoals().get(0));
                planner.getGoals().set(0, newGoal);
                planner.generatePlan();
            } else {
                return new TWThought(TWAction.MOVE, TWDirection.Z);
            }
        }
        // 检查是否生成了计划
        if (!planner.hasPlan()) {
            return RandomMoveThought();
        }

        TWDirection dir = this.planner.execute();
        return new TWThought(TWAction.MOVE, dir);
    }

    @Override
    protected void act(TWThought thought){
        try {
            switch (thought.getAction()) {
                case MOVE:
//				System.out.println("Direction:" + thought.getDirection());
                    move(thought.getDirection());
                    break;
                case PICKUP:
                    TWTile tile = (TWTile) memory.getMemoryGrid().get(this.x, this.y);
                    pickUpTile(tile);
                    planner.getGoals().clear();
                    break;
                case PUTDOWN:
                    TWHole hole = (TWHole) memory.getMemoryGrid().get(this.x, this.y);
                    putTileInHole(hole);
                    planner.getGoals().clear();
                    break;
                case REFUEL:
                    refuel();
                    planner.getGoals().clear();
                    break;
            }
//            this.move(thought.getDirection());

        } catch (CellBlockedException ex) {
//        	System.out.println("Current mode: "+this.mode);
            System.out.println("Size of goal: "+this.planner.getGoals().size());
//        	只能等障碍物消失才能继续前进，重新规划路径？
            System.out.println("N: " + this.memory.isCellBlocked(x, y-1));
            System.out.println("S: " + this.memory.isCellBlocked(x, y+1));
            System.out.println("E: " + this.memory.isCellBlocked(x+1, y));
            System.out.println("W: " + this.memory.isCellBlocked(x-1, y));
            System.out.println("Cell is blocked. Current Position: " + Integer.toString(this.x) + ", " + Integer.toString(this.y));
        }
        System.out.println("Step " + this.getEnvironment().schedule.getSteps());
        System.out.println(name + " score: " + this.score);
//		System.out.println("Assigned Zone: " + Integer.toString(agentZones[agentIdx]));
//		System.out.println("Mode: " + mode.name());
        System.out.println("Position: " + Integer.toString(this.x) + ", " + Integer.toString(this.y));
        System.out.println("Current Mode: " + this.mode);
        System.out.println("Size of goal: "+this.planner.getGoals().size());

        Int2D curGoal = planner.getCurrentGoal();
        if (curGoal != null) {
            System.out.println("Goal: " + curGoal.x + ", " + curGoal.y);
        }
        else
            System.out.println("Goal: Nothing");
        System.out.println("Tiles: " + this.carriedTiles.size());
        System.out.println("Fuel Level: " + this.fuelLevel);
        System.out.println("Fuel Station: " + this.memory.getFuelStation());
        System.out.println("");
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
    public TWAgentWorkingMemory getMemory() {
        return this.memory;
    }

    @Override
    public String getName() {
        return name;
    }
}
