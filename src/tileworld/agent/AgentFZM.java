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
import tileworld.environment.*;
import tileworld.exceptions.CellBlockedException;
import tileworld.planners.*;
import tileworld.agent.*;

import java.util.*;
import java.util.stream.Collectors;

public class AgentFZM extends TWAgent {
    private int index;
    private String name;
    private final int mapWidth = this.getEnvironment().getxDimension();
    private final int mapHeight = this.getEnvironment().getyDimension();
    private State currentState;
    private double fuelThreshold;
    private TWPlannerLZH planner;
    private MyMemory memory;
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

    enum State {
        EXPLORE, COLLECT, FILL, REFUEL, WAIT, FIND_FUELSTATION
    }

    public AgentFZM(int index, String name, int xpos, int ypos, TWEnvironment env, double fuelLevel) {
        super(xpos, ypos, env, fuelLevel);
        this.index = index;
        this.name = name;
        this.fuelThreshold = Math.max(Parameters.xDimension, Parameters.yDimension) * 1.2;
        this.sensor = new TWAgentSensor(this, Parameters.defaultSensorRange);
        this.planner = new TWPlannerLZH(this);
        this.memory = new MyMemory(this, env.schedule, mapWidth, mapHeight);

    }

    @Override
    public void communicate() {
        MyMessage message = new MyMessage(this.getName(), "");
        Bag sensedObjects = new Bag();
        IntBag objectXCoords = new IntBag();
        IntBag objectYCoords = new IntBag();

        this.memory.getMemoryGrid().getNeighborsMaxDistance(x, y, Parameters.defaultSensorRange, false, sensedObjects,
                objectXCoords, objectYCoords);

        System.out.println(this.memory.getFuelStation());
        message.addFuelStationPosition(this.memory.getFuelStation());
        message.addSensedObjects(sensedObjects, new Int2D(x, y));
        System.out.println(message);
        this.getEnvironment().receiveMessage(message);
    }

    private ArrayList<TWEntity> detectObjectNoCollision(PriorityQueue<TWEntity> neighborObjects,
            List<TWAgent> neighbouringAgents) {
        // TODO Auto-generated method stub
        ArrayList<TWEntity> tiles = new ArrayList<TWEntity>();
        for (TWEntity object : neighborObjects) {
            boolean collision = false;
            for (TWAgent agent : neighbouringAgents) {
                // 如果agent离object更近
                //
                // 为什么是/3?
                if (agent.getDistanceTo(object) <= this.getDistanceTo(object) / 3) {
                    collision = true;
                    break;
                }
            }
            if (!collision) {
                tiles.add(object);
            }
        }
        return (tiles.size() > 0) ? tiles : null;
    }

    private Int2D getPositionAdd(Int2D base, Int2D position) {
        return new Int2D(base.x + position.x, base.y + position.y);
    }

    public Int2D findNearestBase(Int2D[] bases, Int2D[] agentPositions) {
        // 使用实际的neighbouringAgents和basePos数组长度初始化代价矩阵
        int agentCount = agentPositions.length;
        int baseCount = bases.length;
        double[][] distances = new double[agentCount][baseCount];

        for (int i = 0; i < agentCount; i++) {
            for (int j = 0; j < baseCount; j++) {
                distances[i][j] = Math.hypot(agentPositions[i].x - bases[j].x, agentPositions[i].y - bases[j].y);
            }
        }

        HungarianAlgorithm algorithm = new HungarianAlgorithm(distances);
        int[] assignments = algorithm.execute();
        Int2D[] optimalBases = Arrays.stream(assignments).mapToObj(i -> bases[i]).toArray(Int2D[]::new);
        return optimalBases[this.index];
    }

    public void addGoalsForFuelStation() {
        System.out.println("Assigning search area for " + this.name);
        planner.voidGoals(); // 清除当前所有目标
        Int2D[] basePos = {
                new Int2D(0, 0),
                new Int2D(0, Parameters.yDimension / 5),
                new Int2D(0, (Parameters.yDimension / 5) * 2),
                new Int2D(0, (Parameters.yDimension / 5) * 3),
                new Int2D(0, (Parameters.yDimension / 5) * 4),
        };

        // 寻找最近的基地
        Int2D nearestBase = findNearestBase(basePos, this.memory.getAgentPositionAll());
        System.out.println(this.name + ", I need to go to: " + nearestBase);

        // 添加目标位置
        // int stripDepth = Parameters.yDimension / 5;
        int sensorRange = Parameters.defaultSensorRange;
        int maxDepth = Parameters.yDimension - sensorRange - 1;
        for (int depth = sensorRange; depth <= maxDepth; depth += sensorRange * 2) {
            int posY = Math.min(depth, maxDepth); // 确保不会超出边界
            Int2D rightEdgePosition = new Int2D(Parameters.xDimension - sensorRange - 1, posY);
            Int2D leftEdgePosition = new Int2D(sensorRange, posY);

            // 添加从基地向右侧和左侧的目标位置
            planner.getGoals().add(getPositionAdd(nearestBase, new Int2D(0, posY)));
            planner.getGoals().add(getPositionAdd(nearestBase, rightEdgePosition));
            if (posY != depth) { // 如果是最后一轮迭代，也添加左边的位置
                planner.getGoals().add(getPositionAdd(nearestBase, leftEdgePosition));
            }
        }
    }

    // 计算指向源点的吸引力
    private void applySource(int x, int y, Int2D sourceAttraction, Map<TWDirection, Double> scores) {
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
    private void applyRepulsion(int x, int y, Map<TWDirection, Double> scores) {
        for (int i = 0; i < this.memory.getAgentPositionAll().length; i++) {
            if (i == this.index)
                continue;
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
        updateDirectionScoresWithHistory(scores);
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
    public void updateRecentLocation(TWDirection selectedDirection) {
        if (recentPosition.size() >= this.recentWindowHistoryLength) {
            recentPosition.poll(); // Remove the oldest
        }
        recentPosition
                .add(getPositionAdd(new Int2D(this.x, this.y), new Int2D(selectedDirection.dx, selectedDirection.dy)));
    }

    // 获取从当前位置到目标位置的方向
    public ArrayList<TWDirection> getDirectionFromPositions(int x1, int y1, int x2, int y2) {
        ArrayList<TWDirection> directions = new ArrayList<>();
        int dx = Integer.compare(x2, x1);
        int dy = Integer.compare(y2, y1);

        if (dx != 0) {
            directions.add(dx > 0 ? TWDirection.E : TWDirection.W);
        }
        if (dy != 0) {
            directions.add(dy > 0 ? TWDirection.S : TWDirection.N);
        }
        return directions;
    }

    private boolean checkMapOutofBound(int x, int y) {
        TWEnvironment env = this.getEnvironment();
        return x < 0 || x >= env.getxDimension() || y < 0 || y >= env.getyDimension();
    }

    private TWThought randomMoveThought() {
        Map<TWDirection, Double> directionScores = new HashMap<>();
        int x = this.getX();
        int y = this.getY();
        System.out.println(this.name + " source: " + sourceAttractionPoint);
        directionScores.put(TWDirection.N, 0.0);
        directionScores.put(TWDirection.S, 0.0);
        directionScores.put(TWDirection.E, 0.0);
        directionScores.put(TWDirection.W, 0.0);

        if (sourceAttractionPoint != null) {
            applySource(x, y, sourceAttractionPoint, directionScores);
        }
        applyRepulsionAndHistory(x, y, directionScores);

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
    private void compareAndSetTarget(TWEntity target) {
        // 判断每个人的距离，如果有人比我更近，就不要去了
        for (TWAgent agent : this.memory.neighbouringAgents) {
            if (agent.getDistanceTo(target) < this.getDistanceTo(target)) {
                return;
            }
        }
        // 如果没有人比我更近，就把这个target加进我的planner里面,
        // 再看是不是在this.memory.getTargetGoalsList里面，如果不是才可以加进planner.getGoals里
        if (!this.memory.getTargetGoalsList().contains(target)) {
            this.planner.getGoals().add(new Int2D(target.getX(), target.getY()));
            // 把target转换成bag, 消息broadcast出去
            Bag targetGoal = new Bag();
            targetGoal.add(target);
            MyMessage message = new MyMessage(this.getName(), "");
            message.addTargetGoal(targetGoal);
        }
    }

    private void compareAndSetTarget(TWEntity target, int index) {
        // 判断每个人的距离，如果有人比我更近，就不要去了
        for (TWAgent agent : this.memory.neighbouringAgents) {
            if (agent.getDistanceTo(target) < this.getDistanceTo(target)) {
                return;
            }
        }
        // 如果没有人比我更近，就把这个target加进我的planner里面,
        // 再看是不是在this.memory.getTargetGoalsList里面，如果不是才可以加进planner.getGoals里
        if (!this.memory.getTargetGoalsList().contains(target)) {
            this.planner.getGoals().add(index, new Int2D(target.getX(), target.getY()));
            // 把target转换成bag, 消息broadcast出去
            Bag targetGoal = new Bag();
            targetGoal.add(target);
            MyMessage message = new MyMessage(this.getName(), "");
            message.addTargetGoal(targetGoal);
            this.getEnvironment().receiveMessage(message);
        }
    }

    private void sendCompletedGoal(TWEntity completedGoalEntity) {
        // Convert the completed goal to a bag
        Bag completedGoal = new Bag();
        completedGoal.add(completedGoalEntity);
        // Send the completed goal to the environment
        MyMessage message = new MyMessage(this.getName(), "");
        message.addCompletedGoal(completedGoal);
        this.getEnvironment().receiveMessage(message);
    }

    private void FindFuelAddGoal(double verysafeFuelThreshold, TWTile targettile, TWHole targethole) {
        // 先检查有没有targettile和targethole, 如果有，就要检查自己和这些东西的距离是不是在我的sensorrange范围里面，
        // 不是直接skip
        if (targettile != null && this.getDistanceTo(targettile) > Parameters.defaultSensorRange) {
            targettile = null;
        }
        if (targethole != null && this.getDistanceTo(targethole) > Parameters.defaultSensorRange) {
            targethole = null;
        }
        addGoalInMiddle(verysafeFuelThreshold, targettile, targethole);

        // 这边要看我的goal()的第一个位置是不是eitherr tile or hole, 如果是就要在这个goal后面插进去我现在的位置，
        // 但是有可能是targettile或者targethole是null, 要先检查
        if (this.planner.getGoals().size() > 0) {
            // 从planner.goal读出第一个goal 的Int2D
            TWObject firstGoal = this.memory.getObject(this.planner.getGoals().get(0).x,
                    this.planner.getGoals().get(0).y);

            if (firstGoal instanceof TWTile || firstGoal instanceof TWHole) {
                this.planner.getGoals().add(1, new Int2D(this.x, this.y));
            }
        }
    }

    private void addGoalInMiddle(double verysafeFuelThreshold, TWTile targettile, TWHole targethole) {

        if (this.planner.getGoals().contains(new Int2D(this.x, this.y))) {
            int index = planner.getGoals().indexOf(new Int2D(this.x, this.y));
            if (index != -1) { // 如果找到了这个目标
                //// 这个需要debug一下
                System.out.println(index);
                planner.getGoals().remove(index);
            }
        }
        // 如果还没有找到加油站，但是fuellevel> remaining_path to go + fuel threshold,
        // 就可以做别的事情（collect or fill)
        else if (this.getFuelLevel() > verysafeFuelThreshold) {
            // 先看有没有有没有同时有tile and hole，如果同时有再看距离
            if (targettile != null && targethole != null) {
                // 如果tile更近
                if (this.carriedTiles.size() < 3 && this.getDistanceTo(targettile.getX(), targettile.getY()) < this
                        .getDistanceTo(targethole.getX(), targethole.getY())) {
                    // 检查去这个地方的step会不会超过上面讲的条件，如果不会：加进goal的第一个位置
                    if (this.getDistanceTo(targettile.getX(), targettile.getY()) + verysafeFuelThreshold < this
                            .getFuelLevel()) {
                        if (!planner.getGoals().isEmpty()
                                && !this.planner.getGoals().contains(new Int2D(targettile.getX(), targettile.getY()))) {
                            this.compareAndSetTarget(targettile, index = 0);
                        }
                    }
                }
                // 如果hole更近
                else {
                    // 检查去这个地方的step会不会超过上面讲的条件，如果不会：加进goal的第一个位置
                    // 还要判断身上有没有tile
                    if (this.carriedTiles.size() > 0 && this.getDistanceTo(targethole.getX(), targethole.getY())
                            + verysafeFuelThreshold < this.getFuelLevel()) {
                        if (!planner.getGoals().isEmpty()
                                && !this.planner.getGoals().contains(new Int2D(targethole.getX(), targethole.getY()))) {
                            this.compareAndSetTarget(targethole, index = 0);
                        }
                    }
                }
            }
            // 如果只有目标tile
            else if (targettile != null && this.carriedTiles.size() < 3) {
                // 检查去这个地方的step会不会超过上面讲的条件，如果不会：加进goal的第一个位置
                if (this.getDistanceTo(targettile.getX(), targettile.getY()) + verysafeFuelThreshold < this
                        .getFuelLevel()) {
                    if (!planner.getGoals().isEmpty()
                            && !this.planner.getGoals().contains(new Int2D(targettile.getX(), targettile.getY()))) {
                        this.compareAndSetTarget(targettile, index = 0);
                    }
                }
            }
            // 如果只有目标hole并且自己carriedTiles.size>0
            else if (targethole != null && this.carriedTiles.size() > 0) {
                // 检查去这个地方的step会不会超过上面讲的条件，如果不会：加进goal的第一个位置
                if (this.getDistanceTo(targethole.getX(), targethole.getY()) + verysafeFuelThreshold < this
                        .getFuelLevel()) {
                    if (!planner.getGoals().isEmpty()
                            && !this.planner.getGoals().contains(new Int2D(targethole.getX(), targethole.getY()))) {
                        this.compareAndSetTarget(targethole, index = 0);
                    }
                }
            }
            // 什么都没有就不用做extra
        }
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

            // 从消息获得别人sensor object位置，感知物体包括位置和种类
            Bag sharedObjects = myMessage.getSensedObjects();
            // 从消息获得agent位置
            Int2D agentPosition = myMessage.getAgentPosition();

            // 从消息得出别人的targetGoal
            Bag targetGoal = myMessage.getTargetGoal();

            // 从消息得出别人已经完成了什么Goal
            Bag completedGoal = myMessage.getCompletedGoal();

            // 从消息得出sourceattraction分布
            Int2D[] sourceAttraction = myMessage.getSourceAttraction();

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
            if (this.memory.getFuelStation() == null && fuelStation != null) {
                System.out.println(this.name + "updating fuel station memory");
                this.memory.setFuelStation(fuelStation.x, fuelStation.y);
                System.out.println(this.memory.getFuelStation());
            }

            // 如果当前消息包含感知物体，则将感知物体和Agent合并一起更新到memory
            if (sharedObjects != null) {
                this.memory.mergeMemory(sharedObjects, agentPosition);
            }

            // 更新大家的targetGoal
            if (targetGoal != null) {
                this.memory.updateTargetGoal(targetGoal);
            }

            // 从大家的CompletedGoal去更新整体的targetGoal
            if (completedGoal != null) {
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
        // TWEntity FuelStation =
        // this.memory.getNearbyObject(x,y,200,TWFuelStation.class);
        // if(FuelStation != null){
        // System.out.println("Fuel Station Found");
        // Scanner scanner = new Scanner(System.in);
        // System.out.println("Press any key to continue...");
        // // Wait for user input
        // scanner.nextLine();
        //
        // System.out.println("Resuming execution.");
        // scanner.close();
        // }
        // TWObject getNearbyObject(int sx, int sy, double threshold, Class<?> type)

        TWTile targettile = null;
        TWHole targethole = null;
        // detect whether collide with other agents or not
        ArrayList<TWEntity> tiles = detectObjectNoCollision(TilesList, this.memory.neighbouringAgents);
        ArrayList<TWEntity> holes = detectObjectNoCollision(HolesList, this.memory.neighbouringAgents);

        if (tiles != null) {
            targettile = (TWTile) tiles.get(0);
        }
        if (holes != null) {
            targethole = (TWHole) holes.get(0);
        }

        // 为什么写进去同一个位置？？？？
        targettile = this.getMemory().getNearbyTile(x, y, 50);
        targethole = this.getMemory().getNearbyHole(x, y, 50);

        /*
         * Decide the currentState based on the current info of agent
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
        // currentState = State.EXPLORE;
        // 先检测是不是刚找到加油站/别人找到了但是自己还在找
        // if(memory.getFuelStation() == null && currentState==State.FIND_FUELSTATION) {
        if (currentState == State.FIND_FUELSTATION && memory.getFuelStation() != null) {
            currentState = State.EXPLORE;
        }

        if (memory.getFuelStation() == null) {
            if (this.getFuelLevel() > this.fuelThreshold) {
                currentState = State.FIND_FUELSTATION;
                System.out.println("Setting currentState to find fuel station");
            } else {
                currentState = State.WAIT;
                System.out.println("No enough fuel, Setting currentState to wait");
            }
        }
        // 如果找到了油站，而且自己要没有油了，就去加油
        // 这边可以设置一个动态阈值
        else if ((memory.getFuelStation() != null) && (this.getFuelLevel() < 2.5
                * this.getDistanceTo(memory.getFuelStation().getX(), memory.getFuelStation().getY()))) {
            currentState = State.REFUEL;
            System.out.println("Setting currentState to refuel");
        }
        // 如果还有油，而且找到了加油站，那就应该pick explore, 除非原本就是REFUEL就不要干扰
        else if (currentState != State.FIND_FUELSTATION && currentState != State.REFUEL) {
            // 如果有tile
            if (this.hasTile()) {
                // 如果身上有少过3个tile
                if (this.carriedTiles.size() < 3) {
                    // 如果有目标hole
                    if (targethole != null) {
                        // 如果有目标tile
                        if (targettile != null) {
                            // 需要判断哪一个更近
                            // 如果目标tile更近
                            if (this.getDistanceTo(targethole.getX(), targethole.getY()) > this
                                    .getDistanceTo(targettile.getX(), targettile.getY())) {
                                currentState = State.COLLECT;
                                System.out.println("Setting currentState to collect");
                            }
                            // 如果目标hole更近
                            else {
                                currentState = State.FILL;
                                System.out.println("Setting currentState to fill");
                            }
                        }
                        // 如果没有目标tile， 但是有Hole 只能去填补了
                        else {
                            currentState = State.FILL;
                            System.out.println("Setting currentState to fill");
                        }
                    }
                    // 如果没有目标hole
                    else {
                        // 如果有目标tile
                        if (targettile != null) {
                            currentState = State.COLLECT;
                            System.out.println("Setting currentState to collect");
                        }
                        // 如果没有目标tile
                        else {
                            currentState = State.EXPLORE;
                            System.out.println("Setting currentState to explore");
                        }
                    }
                }
                // 如果身上有3个tile
                else {
                    // 如果有目标hole
                    if (targethole != null) {
                        currentState = State.FILL;
                        System.out.println("Setting currentState to fill");
                    }
                    // 如果没有目标hole
                    else {
                        currentState = State.EXPLORE;
                        System.out.println("Setting currentState to explore");
                    }
                }
            }
            // 如果完全没有tile
            else {
                // memory里有目标tile吗？
                if (targettile != null) { // this one can check for conflict with other agents
                    currentState = State.COLLECT;
                    System.out.println("Setting currentState to collect");
                } else {
                    currentState = State.EXPLORE;
                }
            }
        }
        Object curLocObject = this.memory.getMemoryGrid().get(x, y);
        // 如果不是空地， 那就直接打印出来是什么
        if (curLocObject != null) {
            System.out.println("Current Location Obj: " + curLocObject.getClass().getName());
        }

        // 如果不是空地，那就是到达了goal或者碰巧经过goal
        ///////////////////////////////////////////////////////////
        // 如果刚好经过加油站，而且剩下的有少过max的75%，就加油
        if (curLocObject instanceof TWFuelStation && (this.getFuelLevel() < (0.7 * Parameters.defaultFuelLevel))) {
            // System.out.println("Current Location is Fuel Station");
            System.out.println("Now Adding Fuel");
            // 加了油就要变成explore
            currentState = State.EXPLORE;
            return new TWThought(TWAction.REFUEL, null);
        } else if (curLocObject instanceof TWHole && this.getEnvironment().canPutdownTile((TWHole) curLocObject, this)
                && this.hasTile()) {
            System.out.println("Filling Hole");
            this.sendCompletedGoal((TWHole) curLocObject);
            return new TWThought(TWAction.PUTDOWN, null);
        } else if (curLocObject instanceof TWTile && this.getEnvironment().canPickupTile((TWTile) curLocObject, this)
                && this.carriedTiles.size() < 3) {
            System.out.println("Picking up Tile");
            this.sendCompletedGoal((TWTile) curLocObject);
            return new TWThought(TWAction.PICKUP, null);
        }
        ///////////////////////////////////////////////////////////

        // 由当前currentState 和Goal 确定 Thought
        // 如果现在的目标是找加油站
        if (currentState == State.FIND_FUELSTATION) {
            // 如果已经做好了path to fuel station就不需要重新路径规划
            // 如果还没有做好规划
            if (this.planner.getGoals().isEmpty() && this.getAllPosition == true) {
                addGoalsForFuelStation();
            } else {
                // 这边是已经做好了寻找加油站的规划
                // 如果目前的位置在goal里面
                double verysafeFuelThreshold = (double) this.planner.getRemainingPathLength()
                        + 2.5 * this.fuelThreshold;
                // 如果走的太远，会导致有些搜索区域没有被搜索到，因此要限制他去拿东西的区域

                FindFuelAddGoal(verysafeFuelThreshold, targettile, targethole);
            }

            for (int i = 0; i < planner.getGoals().size(); i++) {
                System.out.println("Goals " + i + ": " + planner.getGoals().get(i));
            }
            // 已经找到加油站了, 不是FIND_FUELSTATION
        } else {
            planner.voidGoals();
            planner.voidPlan();
            if (currentState == State.REFUEL) {
                double verysafeFuelThreshold = (double) this.planner.getRemainingPathLength()
                        + 1.5 * this.fuelThreshold;
                addGoalInMiddle(verysafeFuelThreshold, targettile, targethole);
                this.planner.getGoals().add(memory.getFuelStation());
            } else if (currentState == State.FILL) {
                this.compareAndSetTarget(targethole);
            } else if (currentState == State.WAIT) {
                return new TWThought(TWAction.MOVE, TWDirection.Z);
            } else if (currentState == State.COLLECT) {
                this.compareAndSetTarget(targettile);
            } else if (currentState == State.EXPLORE) {
                return randomMoveThought();
            }
        }

        // for (int i = 0; i < planner.getGoals().size(); i++) {
        // System.out.println("Goals " + i + ": " + planner.getGoals().get(i));
        // }
        if (this.planner.getGoals().isEmpty()) {
            return randomMoveThought();
        }
        // 如果计划不为空，则调用generatePlan()方法生成一条路径计划
        this.planner.generatePlan();

        if (!planner.hasPlan()) {
            // 如果没有plan但是需要寻找fuelstation(找完了自己的位置但是没有goalStation)
            if (this.currentState == State.FIND_FUELSTATION) {
                Int2D newGoal = generateRandomNearCell(planner.getGoals().get(0));
                planner.getGoals().set(0, newGoal);
                planner.generatePlan();
            } else {
                return new TWThought(TWAction.MOVE, TWDirection.Z);
            }
        }
        // 检查是否生成了计划
        if (!planner.hasPlan()) {
            return randomMoveThought();
        }

        TWDirection dir = this.planner.execute();
        return new TWThought(TWAction.MOVE, dir);
    }

    @Override
    protected void act(TWThought thought) {
        try {
            switch (thought.getAction()) {
                case MOVE:
                    // System.out.println("Direction:" + thought.getDirection());
                    // 不太清楚为什么会out of bound, 检测如果会的话，就随机一个方向
                    if (this.x + thought.getDirection().dx < 0
                            || this.x + thought.getDirection().dx >= this.getEnvironment().getxDimension()
                            || this.y + thought.getDirection().dy < 0
                            || this.y + thought.getDirection().dy >= this.getEnvironment().getyDimension()) {
                        System.out.println("Out of bound");
                        // 随机的方向也不可以outoufbound, 我没有random的function
                        move(randomMoveThought().getDirection());
                    } else {
                        move(thought.getDirection());
                    }
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
            // this.move(thought.getDirection());

        } catch (CellBlockedException ex) {
            // System.out.println("Current currentState: "+this.currentState);
            System.out.println("Size of goal: " + this.planner.getGoals().size());
            // 只能等障碍物消失才能继续前进，重新规划路径？
            System.out.println("N: " + this.memory.isCellBlocked(x, y - 1));
            System.out.println("S: " + this.memory.isCellBlocked(x, y + 1));
            System.out.println("E: " + this.memory.isCellBlocked(x + 1, y));
            System.out.println("W: " + this.memory.isCellBlocked(x - 1, y));
            System.out.println(
                    "Cell is blocked. Current Position: " + Integer.toString(this.x) + ", " + Integer.toString(this.y));
        }
        System.out.println("Step " + this.getEnvironment().schedule.getSteps());
        System.out.println(name + " score: " + this.score);
        // System.out.println("Assigned Zone: " +
        // Integer.toString(agentZones[agentIdx]));
        // System.out.println("State: " + currentState.name());
        System.out.println("Position: " + Integer.toString(this.x) + ", " + Integer.toString(this.y));
        System.out.println("Current State: " + this.currentState);
        System.out.println("Size of goal: " + this.planner.getGoals().size());

        Int2D curGoal = planner.getCurrentGoal();
        if (curGoal != null) {
            System.out.println("Goal: " + curGoal.x + ", " + curGoal.y);
        } else
            System.out.println("Goal: Nothing");
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
