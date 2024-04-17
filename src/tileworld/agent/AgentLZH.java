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
import java.util.ArrayDeque;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.Deque;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.PriorityQueue;
import java.util.Random;
import java.util.List;
import java.util.Map;
import java.util.Scanner;
import java.util.stream.Collectors;

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
    public boolean getAllPosition = false;
    private int updateAllInitialPosition = 0;
    private double RepulsiveConstant = 2;
    private int repulsionRange = 20;
    Deque<Int2D> recentPosition = new LinkedList<>();
    private int recentWindowHistoryLength = 20;
    private int sourceAttraction = 3;
    private Int2D sourceAttractionPoint = null;
    private double RecentRange = 20.0;
    private double recentConstant = 3;

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

    public Int2D findNearestBase(Int2D [] basePos, Int2D [] neighbouringAgents) {
        // This function is to compare distance between agent and other agent , and determine my agent should go which starting point
        int agentCount = neighbouringAgents.length;
        int baseCount = basePos.length;
        double[][] costMatrix = new double[agentCount][baseCount];

        for (int i = 0; i < agentCount; i++){
            for (int j = 0; j < baseCount; j++){
                costMatrix[i][j] = Math.hypot(neighbouringAgents[i].x - basePos[j].x, neighbouringAgents[i].y - basePos[j].y);
            }
        }

        HungarianAlgorithm hungarianAlgorithm = new HungarianAlgorithm(costMatrix);
        int[] result = hungarianAlgorithm.execute();
        // Find the best assignment
        Int2D[] optimalBases = Arrays.stream(result).mapToObj(i -> basePos[i]).toArray(Int2D[]::new);
        return optimalBases[this.index];
    }

    public void addGoalsForFuelStation() {
        System.out.println("Assigning search area for " + this.name);
        planner.voidGoals(); // 清楚当前目标
        Int2D [] basePos = {
            new Int2D(0,0),
            new Int2D(0,Parameters.yDimension/5),
            new Int2D(0,(Parameters.yDimension/5) * 2),
            new Int2D(0,(Parameters.yDimension/5) * 3),
            new Int2D(0,(Parameters.yDimension/5) * 4),
        };
        // Instead of 去到指定的区域，找一个更接近的区域去寻找
        Int2D nearestBase = findNearestBase(basePos, this.memory.getAgentPositionAll());
        System.out.println(this.name + ", I need to go to: " + nearestBase);

        // 添加目标位置
        int sensorRange = Parameters.defaultSensorRange;
        int maxDepth = Parameters.yDimension - sensorRange - 1;
        boolean addRightFirst = true;
        for (int depth = sensorRange; depth <= maxDepth; depth += sensorRange * 2) {
            int posY = Math.min(depth, maxDepth); // 确保不会超出边界
            Int2D rightEdgePosition = new Int2D(Parameters.xDimension - sensorRange - 1, posY);
            Int2D leftEdgePosition = new Int2D(sensorRange, posY);

            // 根据addRightFirst变量决定添加顺序，形成"S形"模式
            if (addRightFirst) {
                planner.getGoals().add(getPositionAdd(nearestBase, rightEdgePosition));
                planner.getGoals().add(getPositionAdd(nearestBase, leftEdgePosition));
            } else {
                planner.getGoals().add(getPositionAdd(nearestBase, leftEdgePosition));
                planner.getGoals().add(getPositionAdd(nearestBase, rightEdgePosition));
            }
            
            if (posY != depth) { // 如果是最后一轮迭代，也添加左边的位置
                if (addRightFirst) {
                    planner.getGoals().add(getPositionAdd(nearestBase, rightEdgePosition));
                } else {
                    planner.getGoals().add(getPositionAdd(nearestBase, leftEdgePosition));
                }
            }
            // Toggle the addition order for the next iteration
            addRightFirst = !addRightFirst;
        }
    }

    // 计算指向源点的吸引力
    private void applySource(int x, int y, Int2D sourceAttraction, Map<TWDirection, Double> scores){
        // 根据自身的位置和需要到达的sourceAttraction的位置，计算出每个方向的score
        double deltaX = x - sourceAttraction.x;
        double deltaY = y - sourceAttraction.y;

        if (deltaX != 0) {
            TWDirection horizontalDir = deltaX > 0 ? TWDirection.W : TWDirection.E;
            scores.merge(horizontalDir, this.sourceAttraction * Math.abs(deltaX), Double::sum);
        }

        if (deltaY != 0) {
            TWDirection verticalDir = deltaY > 0 ? TWDirection.N : TWDirection.S;
            scores.merge(verticalDir, this.sourceAttraction * Math.abs(deltaY), Double::sum);
        }
    }

    // 计算周围其他智能体的排斥力
    private void applyRepulsion(int x, int y, Map<TWDirection, Double> scores){
        for (int i=0; i<5; i++){
            if (i == this.index){
                continue;
            }
            Int2D agentPos = this.memory.getAgentPositionAll()[i];
            double deltaX = x - agentPos.x;
            double deltaY = y - agentPos.y;

            if (Math.abs(deltaX) <= this.repulsionRange) {
                TWDirection horizontalDir = deltaX > 0 ? TWDirection.W : TWDirection.E;
                scores.merge(horizontalDir, -this.RepulsiveConstant * Math.abs(deltaX), Double::sum);
            }

            if (Math.abs(deltaY) <= this.repulsionRange) {
                TWDirection verticalDir = deltaY > 0 ? TWDirection.N : TWDirection.S;
                scores.merge(verticalDir, -this.RepulsiveConstant * Math.abs(deltaY), Double::sum);
            }
        }
    }

    // 结合排斥力和历史移动惩罚更新方向得分
    private void applyRepulsionAndHistory(int x, int y, Map<TWDirection, Double> scores) {
        applyRepulsion(x, y, scores);
        updateDirectionScoresWithHistory(scores);  // Apply a penalty of 1 for recent moves
    }

    // 计算历史移动的惩罚
    private double calculateHistoryPenalty(Int2D newPosition) {
        return recentPosition.stream()
                .mapToDouble(p -> {
                    double distance = newPosition.distance(p);
                    return distance < this.RecentRange ? (this.RecentRange - distance) : 0;
                })
                .sum();
    }

    // 更新方向得分，考虑历史移动
    private void updateDirectionScoresWithHistory(Map<TWDirection, Double> scores) {
        scores.forEach((dir, score) -> {
            Int2D newPosition = new Int2D(this.x + dir.dx, this.y + dir.dy);
            double penalty = calculateHistoryPenalty(newPosition);
            scores.put(dir, score - this.recentConstant * penalty);
        });
    }

    // 更新最近位置记录
    public void updateRecentLocation(TWDirection selectedDirection){
        if (recentPosition.size() >= this.recentWindowHistoryLength) {  // Limit to the last 5 moves
            recentPosition.poll();  // Remove the oldest
        }
        recentPosition.add(getPositionAdd(new Int2D(this.x, this.y), new Int2D(selectedDirection.dx, selectedDirection.dy)));  // Add the newest
    }

    private boolean checkMapOutofBound(int x, int y){
        TWEnvironment env = this.getEnvironment();
        return x < 0 || x >= env.getxDimension() || y < 0 || y >= env.getyDimension();
    }

    private TWThought RandomMoveThought() {
        // ArrayList<TWDirection> dirs = new ArrayList<>();
        Map<TWDirection, Double> directionScores = new HashMap<>();
        int x = this.getX();
        int y = this.getY();
        
        // Int2D sourceAttraction = findNearestBase(this.memory.findLowestNZone(5), this.memory.getAgentPositionAll());
        // print out the agent name + source
        System.out.println(this.name + " source: " + sourceAttraction);

        // Initialize the direction scores
        directionScores.put(TWDirection.N, 0.0);
        directionScores.put(TWDirection.S, 0.0);
        directionScores.put(TWDirection.E, 0.0);
        directionScores.put(TWDirection.W, 0.0);

        if (sourceAttractionPoint != null){
            applySource(x, y, sourceAttractionPoint, directionScores);
        }
        applyRepulsionAndHistory(x, y, directionScores);

        // 调整score所以不会有负数出现
        // Find the minimum score
        double minScore = Collections.min(directionScores.values());
        directionScores.replaceAll((dir, score) -> score + Math.max(-minScore, 0));

        if (!directionScores.isEmpty()) {
            TWDirection selectedDirection = selectDirectionWithWeights(directionScores, x, y);
            updateRecentLocation(selectedDirection);
            return new TWThought(TWAction.MOVE, selectedDirection);
        } else {
            System.out.println("No where to go!");
            return new TWThought(TWAction.MOVE, TWDirection.Z);
        }
    }

    private TWDirection selectDirectionWithWeights(Map<TWDirection, Double> scores, int x, int y) {
        // Filter valid and non-blocked directions with non-zero scores
        List<TWDirection> validDirections = new ArrayList<>();
        List<Double> weights = new ArrayList<>();
    
        scores.forEach((direction, score) -> {
            int newX = x + direction.dx;
            int newY = y + direction.dy;
            if (!checkMapOutofBound(newX, newY) && !this.memory.isCellBlocked(newX, newY)) {
                validDirections.add(direction);
                weights.add(score);
            }
        });

        // If no valid directions are available, return TWDirection.Z
        if (validDirections.isEmpty()) {
            System.out.println("No walkable direction available.");
            return TWDirection.Z;
        }

        return weightedRandomSelection(validDirections, weights);
    }

    private TWDirection weightedRandomSelection(List<TWDirection> directions, List<Double> weights) {
        double totalWeight = weights.stream().mapToDouble(Double::doubleValue).sum();
        if (totalWeight == 0) {
            return directions.get(new Random().nextInt(directions.size()));
        }
        double random = Math.random() * totalWeight;
        double cumulativeWeight = 0.0;
        for (int i = 0; i < directions.size(); i++) {
            cumulativeWeight += weights.get(i);
            if (cumulativeWeight >= random) {
                return directions.get(i);
            }
        }
        return directions.get(directions.size() - 1);
    }


    private Int2D generateRandomNearCell(Int2D goalPos) {
        List<Int2D> potentialPositions = Arrays.asList(
                new Int2D(goalPos.x, goalPos.y - 1),
                new Int2D(goalPos.x, goalPos.y + 1),
                new Int2D(goalPos.x + 1, goalPos.y),
                new Int2D(goalPos.x - 1, goalPos.y));
        List<Int2D> validPositions = potentialPositions.stream()
                .filter(p -> !checkMapOutofBound(p.x, p.y) && !this.memory.isCellBlocked(p.x, p.y))
                .collect(Collectors.toList());

        if (validPositions.isEmpty()) {
            System.out.println("No where to go!");
            return null;
        }
        return validPositions.get(new Random().nextInt(validPositions.size()));
    }

    // 对比然后增加进去goal里面
    private void compareAndSetTarget(TWEntity target, int index) {
        if (this.memory.getTargetGoalsList().contains(target) || isAnyAgentCloser(target)) {
            return;
        }
        // 如果index是有效的，我们会在特定的位置插入目标，否则添加到末尾
        if (index >= 0 && index <= this.planner.getGoals().size()) {
            this.planner.getGoals().add(index, new Int2D(target.getX(), target.getY()));
        } else {
            this.planner.getGoals().add(new Int2D(target.getX(), target.getY()));
        }
        broadcastNewTarget(target);
    }

    private boolean isAnyAgentCloser(TWEntity target) {
        return this.memory.neighbouringAgents.stream()
                .anyMatch(agent -> agent.getDistanceTo(target) < this.getDistanceTo(target));
    }

    private void broadcastNewTarget(TWEntity target) {
        Bag targetGoal = new Bag();
        targetGoal.add(target);
        MyMessage message = new MyMessage(this.getName(), "");
        message.addTargetGoal(targetGoal);
        this.getEnvironment().receiveMessage(message);
    }

    private void compareAndSetTarget(TWEntity target) {
        compareAndSetTarget(target, this.planner.getGoals().size()); // 添加到目标列表末尾
    }

    private void sendCompletedGoal(TWEntity completedGoalEntity){
        // Convert the completed goal to a bag
        Bag completedGoal = new Bag();
        completedGoal.add(completedGoalEntity);
        // Send the completed goal to the environment
        MyMessage message = new MyMessage(this.getName(), "");
        message.addCompletedGoal(completedGoal);
        this.getEnvironment().receiveMessage(message);
    }

    private TWTile checkTileInRange(TWTile tile) {
        return (tile != null && this.getDistanceTo(tile) <= Parameters.defaultSensorRange) ? tile : null;
    }

    private TWHole checkHoleInRange(TWHole hole) {
        return (hole != null && this.getDistanceTo(hole) > Parameters.defaultSensorRange) ? null : hole;
    }

    private void FindFuelAddGoal(double verysafeFuelThreshold, TWTile targettile, TWHole targethole){
        // 先检查有没有targettile和targethole, 如果有，就要检查自己和这些东西的距离是不是在我的sensorrange范围里面， 不是直接skip
        targettile = checkTileInRange(targettile);
        targethole = checkHoleInRange(targethole);
        addGoalInMiddle(verysafeFuelThreshold, targettile, targethole);

        insertCurrentPositionIfNeeded();
        
    }

    private void insertCurrentPositionIfNeeded(){
        // 这边要看我的goal()的第一个位置是不是eitherr tile or hole, 如果是就要在这个goal后面插进去我现在的位置， 但是有可能是targettile或者targethole是null, 要先检查
        if (this.planner.getGoals().size() > 0){
            // 从planner.goal读出第一个goal 的Int2D
            TWObject firstGoal = this.memory.getObject(this.planner.getGoals().get(0).x, this.planner.getGoals().get(0).y);
            if (firstGoal instanceof TWTile || firstGoal instanceof TWHole){
                this.planner.getGoals().add(1, new Int2D(this.x, this.y));
            }
        }
    }

    private void addGoalInMiddle(double verysafeFuelThreshold, TWTile targettile, TWHole targethole){
        if (this.planner.getGoals().contains(new Int2D(this.x, this.y))) {
            this.planner.getGoals().remove(new Int2D(this.x, this.y));
        }
        prioritizeGoal(verysafeFuelThreshold, targettile, targethole);
    }

    private void prioritizeGoal(double verysafeFuelThreshold, TWTile targettile, TWHole targethole){
        // 如果还没有找到加油站，但是fuellevel> remaining_path to go + fuel threshold, 就可以做别的事情（collect or fill)
        if (this.getFuelLevel() > verysafeFuelThreshold){
            // 先看有没有有没有同时有tile and hole，如果同时有再看距离
            // if (targettile!=null && targethole!=null){
            //     boolean isTileCloser = this.getDistanceTo(targettile.getX(), targettile.getY()) < this.getDistanceTo(targethole.getX(), targethole.getY());
            //     if (this.carriedTiles.size()< 3 && shouldPrioritizeGoal(targettile, verysafeFuelThreshold)){
            //         this.compareAndSetTarget(targettile, 0);
            //     }
            //     else if (!isTileCloser && this.carriedTiles.size() > 0 && shouldPrioritizeGoal(targettile, verysafeFuelThreshold)){
            //         this.compareAndSetTarget(targethole, 0);
            //     }
            // }
            // // 如果只有目标tile
            // else if (targettile != null && this.carriedTiles.size() < 3 && shouldPrioritizeGoal(targettile, verysafeFuelThreshold)) {
            //     this.compareAndSetTarget(targettile, 0);
            // } else if (targethole != null && this.carriedTiles.size() > 0 && shouldPrioritizeGoal(targethole, verysafeFuelThreshold)) {
            //     this.compareAndSetTarget(targethole, 0);
            // }
            // // 什么都没有就不用做extra
            if (targettile != null && this.carriedTiles.size() < 3 && shouldPrioritizeGoal(targettile, verysafeFuelThreshold)) {
                this.compareAndSetTarget(targettile, 0);
            } else if (targethole != null && this.carriedTiles.size() > 0 && shouldPrioritizeGoal(targethole, verysafeFuelThreshold)) {
                this.compareAndSetTarget(targethole, 0);
            }
        }
    }


    private boolean shouldPrioritizeGoal(TWEntity entity, double threshold) {
        return this.getDistanceTo(entity.getX(), entity.getY()) + threshold < this.getFuelLevel();
    }

    
    @Override
    protected TWThought think() {
        /** 
         * get message and update memory
         */
        ArrayList<Message> received_message = this.getEnvironment().getMessages();
        for (Message message : received_message) {
            // System.out.println(message);
            MyMessage myMessage = (MyMessage) message;
            Int2D fuelStation = myMessage.getFuelStationPosition();
            
            //从消息获得别人sensor object位置，感知物体包括位置和种类
            Bag sharedObjects = myMessage.getSensedObjects();
            //从消息获得agent位置
            Int2D agentPosition = myMessage.getAgentPosition();

            // 从消息得出别人的targetGoal
            Bag targetGoal = myMessage.getTargetGoal();
    
            // 从消息得出别人已经完成了什么Goal
            Bag completedGoal = myMessage.getCompletedGoal();

            // 从消息得出sourceattraction分布
            Int2D [] sourceAttraction = myMessage.getSourceAttraction();

            // 记录每个agent的初始位置， 用于确定搜索区域
            String numberOnly = message.getFrom().replaceAll("[^\\d]", "");
            int agentIndex = Integer.parseInt(numberOnly) - 1;
            if (agentPosition != null) {
                // this.memory.agentInitialPos[agentIndex] = agentPosition;
                // 从消息得到agent的位置，更新memory
                if (this.memory.getAgentPosition(agentIndex) == null) {
                    this.updateAllInitialPosition++;
                }
                this.memory.updateAgentPosition(agentIndex, agentPosition);
                // update heatmap (visitcount)
                this.memory.recordVisit(agentPosition.getX(), agentPosition.getY());
            }
            if (this.updateAllInitialPosition == 5) {
                this.getAllPosition = true;
            }

            // 先看sourceAttraction是不是null, 再获取自己的sourceAttraction地点
            if (sourceAttraction != null) {
                sourceAttractionPoint = sourceAttraction[this.index];
            }
            
            // 如果没有消息或者发送者是自己，跳过
            if (message == null || message.getFrom().equals(this.getName())) {
                continue;
            }
            
            // 如果自身memory没有fuelstation位置和消息中有fuelstation位置，更新memory
            if(this.memory.getFuelStation()== null && fuelStation != null) {
                System.out.println(this.name + "updating fuel station memory");
                this.memory.setFuelStation(fuelStation.x, fuelStation.y);
                System.out.println(this.memory.getFuelStation());
            }

            // 如果当前消息包含感知物体，则将感知物体和Agent合并一起更新到memory
            if (sharedObjects != null) {
                this.memory.mergeMemory(sharedObjects, agentPosition);
            }

            // 更新大家的targetGoal
            if (targetGoal != null){
                this.memory.updateTargetGoal(targetGoal);
            }

            // 从大家的CompletedGoal去更新整体的targetGoal
            if (completedGoal != null){
                this.memory.removeTargetGoal(completedGoal);
            }

        }

        // 更新memory heatmap
        // this.memory.recordVisit(this.x, this.y);

        // 根据agent内存中获取附近的对象，并使用优先级队列按照它们与agent的距离进行排序
        PriorityQueue<TWEntity> TilesList;
        PriorityQueue<TWEntity> HolesList;
        // PriorityQueue<TWEntity> FuelStation;
        // 利用欧几里得距离排序对象与agent的距离

        TilesList = this.memory.getNearbyAllSortedObjects(x, y, 50, TWTile.class);
        HolesList = this.memory.getNearbyAllSortedObjects(x, y, 50, TWHole.class);
        // TWEntity FuelStation = this.memory.getNearbyObject(x,y,200,TWFuelStation.class);
//        if(FuelStation != null){
//            System.out.println("Fuel Station Found"); 
//            Scanner scanner = new Scanner(System.in);
//            System.out.println("Press any key to continue...");
//            // Wait for user input
//            scanner.nextLine();
//
//            System.out.println("Resuming execution.");
//            scanner.close();
//        }
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
        // 逻辑思路 现决定模式，再决定Thought
        // mode = Mode.EXPLORE;
//        先检测是不是刚找到加油站/别人找到了但是自己还在找
        if (mode==Mode.FIND_FUELSTATION && memory.getFuelStation()!= null){
        	mode = Mode.EXPLORE;
        }
        
        if (memory.getFuelStation() == null) {
            if (this.getFuelLevel() > this.fuelThreshold) {
                mode = Mode.FIND_FUELSTATION;
                System.out.println("Setting mode to find fuel station");
            }
            else {
                mode = Mode.WAIT;
                System.out.println("No enough fuel, Setting mode to wait");
            }
        }
        // 如果找到了油站，而且自己要没有油了，就去加油
        // 这边可以设置一个动态阈值
        else if ((memory.getFuelStation() != null) && (this.getFuelLevel() < 2.5* this.getDistanceTo(memory.getFuelStation().getX(), memory.getFuelStation().getY()))){
            mode = Mode.REFUEL;
            System.out.println("Setting mode to refuel");
        }
        // 如果还有油，而且找到了加油站，那就应该pick explore, 除非原本就是REFUEL就不要干扰
        else if (mode!= Mode.FIND_FUELSTATION && mode!=Mode.REFUEL){
            // 如果有tile
            if (this.hasTile()){
                // 如果身上有少过3个tile
                if (this.carriedTiles.size() <3){
                    // 如果有目标hole
                    if (targethole != null){
                        // 如果有目标tile
                        if(targettile != null){
                            // 需要判断哪一个更近
                            // 如果目标tile更近
                            if (this.getDistanceTo(targethole.getX(), targethole.getY()) > this.getDistanceTo(targettile.getX(), targettile.getY())){
                                mode = Mode.COLLECT;
                                System.out.println("Setting mode to collect");
                            }
                            // 如果目标hole更近
                            else{
                                mode = Mode.FILL;
                                System.out.println("Setting mode to fill");
                            }
                        }
                        // 如果没有目标tile， 但是有Hole 只能去填补了
                        else {
                            mode = Mode.FILL;
                            System.out.println("Setting mode to fill");
                        }
                    }
                    // 如果没有目标hole
                    else{
                        //如果有目标tile
                        if (targettile != null){
                            mode = Mode.COLLECT;
                            System.out.println("Setting mode to collect");
                        }
                        // 如果没有目标tile
                        else{
                            mode = Mode.EXPLORE;
                            System.out.println("Setting mode to explore");  
                        } 
                    }
                }
                // 如果身上有3个tile
                else{
                    // 如果有目标hole
                    if (targethole != null){
                        mode = Mode.FILL;
                        System.out.println("Setting mode to fill");
                    }
                    // 如果没有目标hole
                    else{
                        mode = Mode.EXPLORE;
                        System.out.println("Setting mode to explore");
                    }
                }
            }
            // 如果完全没有tile
            else{
                // memory里有目标tile吗？
                if (targettile != null) { // this one can check for conflict with other agents
                    mode = Mode.COLLECT;
                    System.out.println("Setting mode to collect");
                }
                else {mode = Mode.EXPLORE;} 
            }
        } 
        Object curLocObject = this.memory.getMemoryGrid().get(x, y);
        // 如果不是空地， 那就直接打印出来是什么
        if (curLocObject != null){
            System.out.println("Current Location Obj: " + curLocObject.getClass().getName());   
        }

        // 如果不是空地，那就是到达了goal或者碰巧经过goal
        ///////////////////////////////////////////////////////////
        // 如果刚好经过加油站，而且剩下的有少过max的75%，就加油
        if (curLocObject instanceof TWFuelStation && (this.getFuelLevel() < (0.7 * Parameters.defaultFuelLevel))){
            // System.out.println("Current Location is Fuel Station");
            System.out.println("Now Adding Fuel");
            // 加了油就要变成explore
            mode = Mode.EXPLORE;
            return new TWThought(TWAction.REFUEL, null);
        }
        else if (curLocObject instanceof TWHole && this.getEnvironment().canPutdownTile((TWHole)curLocObject,this) && this.hasTile()){
            System.out.println("Filling Hole");
            this.sendCompletedGoal((TWHole)curLocObject);
            return new TWThought(TWAction.PUTDOWN, null);
        }
        else if (curLocObject instanceof TWTile && this.getEnvironment().canPickupTile((TWTile)curLocObject, this) && this.carriedTiles.size()<3){
            System.out.println("Picking up Tile");
            this.sendCompletedGoal((TWTile)curLocObject);
            return new TWThought(TWAction.PICKUP, null);
        }
        ///////////////////////////////////////////////////////////

        // 由当前mode 和Goal 确定 Thought
        // 如果现在的目标是找加油站
        if (mode == Mode.FIND_FUELSTATION){
            // 如果已经做好了path to fuel station就不需要重新路径规划
            // 如果还没有做好规划
            if(this.planner.getGoals().isEmpty() && this.getAllPosition == true){
                addGoalsForFuelStation();
            }else{ 
                // 这边是已经做好了寻找加油站的规划
                // 如果目前的位置在goal里面
                double verysafeFuelThreshold = (double)this.planner.getRemainingPathLength() + 2.5 * this.fuelThreshold;
                // 如果走的太远，会导致有些搜索区域没有被搜索到，因此要限制他去拿东西的区域
                
                FindFuelAddGoal(verysafeFuelThreshold, targettile, targethole);
            }

            for (int i = 0; i<planner.getGoals().size();i++){
                System.out.println("Goals " + i + ": " + planner.getGoals().get(i));
            }
        // 已经找到加油站了, 不是FIND_FUELSTATION
        }else{
            planner.voidGoals();
            planner.voidPlan();
            if (mode == Mode.REFUEL){
                double verysafeFuelThreshold = (double)this.planner.getRemainingPathLength() + 1.5 * this.fuelThreshold;
                addGoalInMiddle(verysafeFuelThreshold, targettile, targethole);
                this.planner.getGoals().add(memory.getFuelStation());
            }
            else if(mode == Mode.FILL){
                this.compareAndSetTarget(targethole);
            }
            else if (mode == Mode.WAIT){
                return new TWThought(TWAction.MOVE, TWDirection.Z);
            }
            else if (mode == Mode.COLLECT){
                this.compareAndSetTarget(targettile);
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
        //如果计划不为空，则调用generatePlan()方法生成一条路径计划
        this.planner.generatePlan();

        if (!planner.hasPlan()) {
            // 如果没有plan但是需要寻找fuelstation(找完了自己的位置但是没有goalStation)
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

    private void performMoveAction(TWDirection direction) {
        int newX = this.x + direction.dx;
        int newY = this.y + direction.dy;
        if (checkMapOutofBound(newX, newY)) {
            System.out.println("Out of bound");
            // Attempt to recover from out of bounds by moving in a valid random direction
            try {
                move(RandomMoveThought().getDirection());
            } catch (CellBlockedException e) {
                handleBlockedCell(); // Handle the case where even the random direction is blocked
            }
        } else {
            try {
                move(direction);
            } catch (CellBlockedException e) {
                handleBlockedCell(); // Handle blocked cell situation
            }
        }
    }

    @Override
    protected void act(TWThought thought){
        switch (thought.getAction()) {
            case MOVE:
                performMoveAction(thought.getDirection());
                break;
            case PICKUP:
                TWTile tile = (TWTile) memory.getMemoryGrid().get(this.x, this.y);
                pickUpTile(tile);
                // planner.getGoals().clear();
                planner.getGoals().remove(new Int2D(this.x, this.y));
                break;
            case PUTDOWN:
                TWHole hole = (TWHole) memory.getMemoryGrid().get(this.x, this.y);
                putTileInHole(hole);
                // planner.getGoals().clear();
                planner.getGoals().remove(new Int2D(this.x, this.y));
                break;
            case REFUEL:
                refuel();
                planner.getGoals().clear();
                break;
        }
        printAgentState();
    }

    private void handleBlockedCell() {
        System.out.println("Size of goal: " + this.planner.getGoals().size());
        System.out.println("N: " + this.memory.isCellBlocked(x, y - 1));
        System.out.println("S: " + this.memory.isCellBlocked(x, y + 1));
        System.out.println("E: " + this.memory.isCellBlocked(x + 1, y));
        System.out.println("W: " + this.memory.isCellBlocked(x - 1, y));
        System.out.println("Cell is blocked. Current Position: " + this.x + ", " + this.y);
    }
    
    private void printAgentState() {
        System.out.println("Step " + this.getEnvironment().schedule.getSteps());
        System.out.println(name + " score: " + this.score);
        System.out.println("Position: " + this.x + ", " + this.y);
        System.out.println("Current State: " + this.mode);
        System.out.println("Size of goal: " + this.planner.getGoals().size());
        Int2D curGoal = planner.getCurrentGoal();
        System.out.println("Goal: " + (curGoal != null ? curGoal.x + ", " + curGoal.y : "Nothing"));
        System.out.println("Tiles: " + this.carriedTiles.size());
        System.out.println("Fuel Level: " + this.fuelLevel);
        System.out.println("Fuel Station: " + this.memory.getFuelStation());
        System.out.println("");
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
