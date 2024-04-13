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
import java.util.PriorityQueue;
import java.util.Random;
import java.util.List;
import java.util.Map;
import java.util.Scanner;

import tileworld.agent.HungarianAlgorithm;

public class AgentLZHInspector extends AgentLZH{

    public AgentLZHInspector(int index, String name, int xpos, int ypos, TWEnvironment env, double fuelLevel) {
        super(index, name, xpos, ypos, env, fuelLevel);
        //TODO Auto-generated constructor stub
    }
    
    // 计算heatmap再把分布的点发出去
    public Int2D[] findNearestBaseList(Int2D [] basePos, Int2D [] neighbouringAgents) {
        // This function is to compare distance between agent and other agent , and determine my agent should go which starting point
        // Int2D myagentPosition = new Int2D(this.getX(), this.getY());
        int numAgents = 5;
        int numBases = 5;
        double [][] costMatrix = new double[numAgents][numBases];

        for (int i = 0; i < numAgents; i++){
            // TWAgent agent = neighbouringAgents.get(i);
            for (int j = 0; j < numBases; j++){
                costMatrix[i][j] = Math.abs(neighbouringAgents[i].x-basePos[j].x) + Math.abs(neighbouringAgents[i].y-basePos[j].y);
                // costMatrix[i][j] = agent.getDistanceTo(basePos[j].x, basePos[j].y);
            }
        }

        // Apply Hungarian Algorithm to find the best assignment
        HungarianAlgorithm hungarianAlgorithm = new HungarianAlgorithm(costMatrix);
        int[] result = hungarianAlgorithm.execute();

        // Find the best assignment
        Int2D[] optimalBases = new Int2D[numAgents];
        for (int i = 0; i<result.length; i++){
            optimalBases[i] = basePos[result[i]];
        }
        return optimalBases;
    }

    // Override the communication to calculate the heatmap and broadcast the point
    @Override
    public void communicate() {
        MyMessage message = new MyMessage(this.getName(), "");
        Bag sensedObjects = new Bag();
        IntBag objectXCoords = new IntBag();
        IntBag objectYCoords = new IntBag();

        // 从sensedobject中更新memory
        this.getMemory().getMemoryGrid().getNeighborsMaxDistance(x, y, Parameters.defaultSensorRange, false, sensedObjects, objectXCoords, objectYCoords);
        // System.out.print
        
        // 发送消息位置
        // 油站位置
        System.out.println(((MyMemory) this.getMemory()).getFuelStation());
        message.addFuelStationPosition(((MyMemory) this.getMemory()).getFuelStation());

        // sensedObjects位置和自身位置
        message.addSensedObjects(sensedObjects, new Int2D(x, y));
        System.out.println(message);

        // 先检查是不是全部agent location都拿到了
        if (this.getAllPosition){
           ((MyMemory) this.getMemory()).decayHeatMap();
            Int2D []  sourceAttractionList = findNearestBaseList(((MyMemory) this.getMemory()).findLowestNZone(5), ((MyMemory) this.getMemory()).getAgentPositionAll());
            message.addSourceAttraction(sourceAttractionList);
        }
    
        // 发到environment
        this.getEnvironment().receiveMessage(message);
    }

}