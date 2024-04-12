/*
 * To change this template, choose Tools | Templates
 * and open the template in the editor.
 */
package tileworld.planners;

import sim.util.Int2D;
import tileworld.environment.TWDirection;

import java.util.ArrayList;
import tileworld.Parameters;
import tileworld.agent.TWAgent;
import tileworld.environment.TWDirection;


/**
 * DefaultTWPlanner
 *
 * @author michaellees
 * Created: Apr 22, 2010
 *
 * Copyright michaellees 2010
 *
 * Here is the skeleton for your planner. Below are some points you may want to
 * consider.
 *
 * Description: This is a simple implementation of a Tileworld planner. A plan
 * consists of a series of directions for the agent to follow. Plans are made,
 * but then the environment changes, so new plans may be needed
 *
 * As an example, your planner could have 4 distinct behaviors:
 *
 * 1. Generate a random walk to locate a Tile (this is triggered when there is
 * no Tile observed in the agents memory
 *
 * 2. Generate a plan to a specified Tile (one which is nearby preferably,
 * nearby is defined by threshold - @see TWEntity)
 *
 * 3. Generate a random walk to locate a Hole (this is triggered when the agent
 * has (is carrying) a tile but doesn't have a hole in memory)
 *
 * 4. Generate a plan to a specified hole (triggered when agent has a tile,
 * looks for a hole in memory which is nearby)
 *
 * The default path generator might use an implementation of A* for each of the behaviors
 *
 */
public class TWPlannerLZH implements TWPlanner {
    private TWAgent agent;
    private TWPath plan;
    private ArrayList<Int2D> goals;
    private AstarPathGenerator pathGenerator;

    public TWPlannerLZH(TWAgent agent) {
        this.agent = agent;
        this.plan = null;
        this.goals = new ArrayList<>(0);
        this.pathGenerator = new AstarPathGenerator(agent.getEnvironment(), agent, Parameters.xDimension+ Parameters.yDimension);
    }
    public TWPath generatePlan() {
        // throw new UnsupportedOperationException("Not supported yet.");
        int startX = agent.getX();
        int startY = agent.getY();
        int goalX = goals.get(0).x;
        int goalY = goals.get(0).y;
        if(!goals.isEmpty()){
            if(this.agent.getEnvironment().isInBounds(goalX, goalY)){
               this.plan = pathGenerator.findPath(startX, startY, goalX, goalY);
            }
        }
        return plan;
    }

    @Override
    public boolean hasPlan() {
        // throw new UnsupportedOperationException("Not supported yet.");
        return (plan != null) ? plan.hasNext() : false;
    }

    public void voidPlan() {
        // throw new UnsupportedOperationException("Not supported yet.");
        plan = null;
    }

    public Int2D getCurrentGoal() {
        // throw new UnsupportedOperationException("Not supported yet.");
        return (goals.isEmpty())   ? null : goals.get(0);

    }

    public TWDirection execute() {
        // throw new UnsupportedOperationException("Not supported yet.");
        return plan.popNext().getDirection();
    }

    public void printPlan() {
        System.out.println("path"+ plan);
    }

    public ArrayList<Int2D> getGoals(){
        return goals;
    }
    public void voidGoals(){
        goals.clear();
    }
    public int getRemainingPathLength(){
        // 根据goal list，串联每一个goal，输出还需要多少的步数
        if (goals == null || goals.isEmpty()){
            return 0;
        }
        int totalPathLength = 0;
        int startX = agent.getX();
        int startY = agent.getY();
        int currentX = startX;
        int currentY = startY;
        for (Int2D goal: goals){
            if(this.agent.getEnvironment().isInBounds(goal.x, goal.y)){
                TWPath path = pathGenerator.findPath(currentX, currentY, goal.x, goal.y);
                if(path != null){
                    totalPathLength += (path.getpath().size() - 1);
                    currentX = goal.x;
                    currentY = goal.y;
                }
            }
                
        }

        return totalPathLength;
    }
    

}

