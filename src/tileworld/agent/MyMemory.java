package tileworld.agent;

import java.util.ArrayList;
import java.util.Comparator;
import java.util.HashMap;
import java.util.List;
import java.util.PriorityQueue;

import sim.engine.Schedule;
import sim.field.grid.ObjectGrid2D;
import sim.util.Bag;
import sim.util.Int2D;
import sim.util.IntBag;
import tileworld.Parameters;
import tileworld.environment.NeighbourSpiral;
import tileworld.environment.TWEntity;
import tileworld.environment.TWFuelStation;
import tileworld.environment.TWHole;
import tileworld.environment.TWObject;
import tileworld.environment.TWObstacle;
import tileworld.environment.TWTile;

public class MyMemory extends TWAgentWorkingMemory {
	protected Schedule schedule;
	protected TWAgent me;
	protected final static int MAX_TIME = Parameters.lifeTime;
	public ObjectGrid2D memoryGrid;
//	private final static float MEM_DECAY = 0.5f;
	public TWAgentPercept[][] objects;
	public int memorySize;
	protected HashMap<Class<?>, TWEntity> closestInSensorRange;
	static protected List<Int2D> spiral = new NeighbourSpiral(Parameters.defaultSensorRange * 4).spiral();
	protected List<TWAgent> neighbouringAgents = new ArrayList<TWAgent>();

	protected Int2D fuelStation;
	
	protected Int2D getFuelStation() {
		return fuelStation;
	}
	protected void setFuelStation(int x, int y) {
		this.fuelStation = new Int2D(x, y); 
	}

	public MyMemory(TWAgent moi, Schedule schedule, int x, int y) {
		super(moi, schedule, x, y);
		closestInSensorRange = new HashMap<Class<?>, TWEntity>(4);
//		this.sensedMemory = new TWAgentPercept[Parameters.defaultSensorRange * 2 + 1][Parameters.defaultSensorRange * 2 + 1];
		this.me = moi;

		this.objects = new TWAgentPercept[x][y];

		this.schedule = schedule;
		this.memoryGrid = new ObjectGrid2D(me.getEnvironment().getxDimension(), me.getEnvironment().getyDimension());
		
//		this.fuelStation = null;
		// TODO Auto-generated constructor stub
	}
	public void mergeMemory(Bag sensedObjects, Int2D posAgent) {
		// assert (sensedObjects.size() == objectXCoords.size() && sensedObjects.size() == objectYCoords.size());
		/*
		 * clear memory.objetcs centered agent (posX,posY), for example, 7 * 7 
		 * */
		int minX = Math.max(0, posAgent.getX() - Parameters.defaultSensorRange);
		int maxX = Math.min(Parameters.xDimension, posAgent.getX() + Parameters.defaultSensorRange);
		int minY = Math.max(0, posAgent.getY() - Parameters.defaultSensorRange);
		int maxY = Math.min(Parameters.yDimension, posAgent.getY() + Parameters.defaultSensorRange);

		for (int i = minX; i < maxX; i++) {
			for(int j = minY; j < maxY; j++) {
				// System.out.println(i + "," + j);
				if (objects[i][j] != null) {
					objects[i][j] = null;
					memoryGrid.set(i, j, null);
					memorySize--;
				}
			}
		}
		
		/*
		 * For new objects the agent sensed
		 * */
		for (int i = 0; i < sensedObjects.size(); i++) {
			TWEntity o = (TWEntity) sensedObjects.get(i);
			if (!(o instanceof TWEntity)) {
				// System.out.println("Not an TWEntity!");	
				continue;
			}
			//store the fuel station  
			if (this.fuelStation == null && o instanceof TWFuelStation) {
				// System.out.println(this.me.getName() + ": I found fuel station!!  " + o.getX() + ", " + o.getY());
				setFuelStation(o.getX(), o.getY());
			}				
			//if nothing in memory currently, then were increasing the number 
			//of items we have in memory by 1			
			//if(objects[o.getX()][o.getY()] == null) memorySize++;
			
			//Add the object to memory
			objects[o.getX()][o.getY()] = new TWAgentPercept(o, this.getSimulationTime());
			memoryGrid.set(o.getX(), o.getY(), o);
//			updateClosest(o);
		}
	}
	/**
	 * 
	 * 
	 * /
	
	/**
	 * Called at each time step, updates the memory map of the agent.
	 * Note that some objects may disappear or be moved, in which case part of
	 * sensed may contain null objects
	 *
	 * Also note that currently the agent has no sense of moving objects, so
	 * an agent may remember the same object at two locations simultaneously.
	 * 
	 * Other agents in the grid are sensed and passed to this function. But it
	 * is currently not used for anything. Do remember that an agent sense itself
	 * too.
	 *
	 * @param sensedObjects bag containing the sensed objects
	 * @param objectXCoords bag containing x coordinates of objects
	 * @param objectYCoords bag containing y coordinates of object
	 * @param sensedAgents bag containing the sensed agents
	 * @param agentXCoords bag containing x coordinates of agents
	 * @param agentYCoords bag containing y coordinates of agents
	 */
	@Override
	public void updateMemory(Bag sensedObjects, IntBag objectXCoords, IntBag objectYCoords, Bag sensedAgents, IntBag agentXCoords, IntBag agentYCoords) {
		// throw new UnsupportedOperationException("I found fuel station!!");
		// System.out.println(this.me.getName()+ ": Updating my memory........");	
		
		//reset the closest objects for new iteration of the loop (this is short
		//term observation memory if you like) It only lasts one timestep
		// not used
		closestInSensorRange = new HashMap<Class<?>, TWEntity>(4);
		
		//must all be same size.
		assert (sensedObjects.size() == objectXCoords.size() && sensedObjects.size() == objectYCoords.size());

		// me.getEnvironment().getMemoryGrid().clear();  // THis is equivalent to only having sensed area in memory
		this.decayMemory();       // You might want to think about when to call the decay function as well.
		
		/*
		 * clear memory.objetcs centered agent (posX,posY), for example, 7 * 7 
		 * */
		int minX = Math.max(0, me.getX() - Parameters.defaultSensorRange);
		int maxX = Math.min(Parameters.xDimension, me.getX() + Parameters.defaultSensorRange);
		int minY = Math.max(0, me.getY() - Parameters.defaultSensorRange);
		int maxY = Math.min(Parameters.yDimension, me.getY() + Parameters.defaultSensorRange);

		for (int i = minX; i < maxX; i++) {
			for(int j = minY; j < maxY; j++) {
				// System.out.println(i + "," + j);
				if (objects[i][j] != null) {
					objects[i][j] = null;
					memoryGrid.set(i, j, null);
					memorySize--;
				}
			}
		}
		
		/*
		 * For new objects sensed
		 * */
		for (int i = 0; i < sensedObjects.size(); i++) {
			TWEntity o = (TWEntity) sensedObjects.get(i);
			if (!(o instanceof TWEntity)) {
				// System.out.println("Not an TWEntity!");	
				continue;
			}
			if (o instanceof TWObstacle) {
//				System.out.println(this.me.getName() + ": An obstable!!  " + o.getX() + ", " + o.getY());
			}
			//判断是否是fuel station  
			if (this.fuelStation == null && o instanceof TWFuelStation) {
				System.out.println(this.me.getName() + ": I found fuel station!!  " + o.getX() + ", " + o.getY());
				setFuelStation(o.getX(), o.getY());
//				 throw new UnsupportedOperationException("I found fuel station!!");
			}				
			//if nothing in memory currently, then were increasing the number 
			//of items we have in memory by 1			
//			if(objects[o.getX()][o.getY()] == null) memorySize++;
			
			//Add the object to memory
			objects[o.getX()][o.getY()] = new TWAgentPercept(o, this.getSimulationTime());
			memoryGrid.set(o.getX(), o.getY(), o);
			updateClosest(o);
		}
		
        // Agents are currently not added to working memory. Depending on how 
        // communication is modelled you might want to do this.
        neighbouringAgents.clear();
		for (int i = 0; i < sensedAgents.size(); i++) {
            if (!(sensedAgents.get(i) instanceof TWAgent)) {
                assert false;
            }
            TWAgent a = (TWAgent) sensedAgents.get(i);
            if(a == null || a.equals(me)){
                continue;
            }
            neighbouringAgents.add(a);
        }
	}
	
	public TWAgent getNeighbour(){
        if(neighbouringAgents.isEmpty()){
            return null;
        }else{
            return neighbouringAgents.get(0);
        }
    }
	
	@Override
	public void updateMemory(TWEntity[][] sensed, int xOffset, int yOffset) {
		for (int x = 0; x < sensed.length; x++) {
			for (int y = 0; y < sensed[x].length; y++) {
				objects[x + xOffset][y + yOffset] = new TWAgentPercept(sensed[x][y], this.getSimulationTime());
			}
		}
	}
	@Override
	public void decayMemory() {
		// put some decay on other memory pieces (this will require complete
		// iteration over memory though, so expensive.
		//This is a simple example of how to do this.
	    for (int x = 0; x < this.objects.length; x++) {
	       for (int y = 0; y < this.objects[x].length; y++) {
	           TWAgentPercept currentMemory =  objects[x][y];
//	           currentMemory.getO().getEnvironment().
	           if(currentMemory!=null && !(currentMemory.getO() instanceof TWFuelStation) && currentMemory.getT() < schedule.getTime()-MAX_TIME){ 
	        	   objects[x][y] = null;
	        	   memoryGrid.set(x, y, null);
	               memorySize--;
	           }
	       }
	    }
	}
	
	@Override
	public ObjectGrid2D getMemoryGrid() {
		return this.memoryGrid;
	}
	@Override
	public boolean isCellBlocked(int tx, int ty) {
		// no memory at all, so assume not blocked
		if (objects[tx][ty] == null) {
			return false;
		}

		TWEntity e = objects[tx][ty].getO();
		// is it an obstacle?
		return (e instanceof TWObstacle);
	}
	@Override
	public void removeAgentPercept(int x, int y){
		objects[x][y] = null;
		memoryGrid.set(x, y, null);
	}
	@Override
	public void removeObject(TWEntity o){
		removeAgentPercept(o.getX(), o.getY());
	}
	protected double getSimulationTime() {
		return schedule.getTime();
	}
	protected void updateClosest(TWEntity o) {
		assert (o != null);
		if (closestInSensorRange.get(o.getClass()) == null || me.closerTo(o, closestInSensorRange.get(o.getClass()))) {
			closestInSensorRange.put(o.getClass(), o);
			
		}
	}

	/**
	 * Returns the nearest object that has been remembered recently where recently
	 * is defined by a number of timesteps (threshold)
	 *
	 * If no Object is in memory which has been observed in the last threshold
	 * timesteps it returns the most recently observed object. If there are no objects in
	 * memory the method returns null. Note that specifying a threshold of one
	 * will always return the most recently observed object. Specifying a threshold
	 * of MAX_VALUE will always return the nearest remembered object.
	 *
	 * Also note that it is likely that nearby objects are also the most recently observed
	 *
	 *
	 * @param x coordinate from which to check for objects
	 * @param y coordinate from which to check for objects
	 * @param threshold how recently we want to have seen the object
	 * @param type the class of object we're looking for (Must inherit from TWObject, specifically tile or hole)
	 * @return
	 */
	
	class ObjectComparator implements Comparator<TWEntity>{
        
        // Overriding compare()method of Comparator 
                    // for descending order of cgpa
        public int compare(TWEntity o1, TWEntity o2) {
            int dis1 = (int) me.getDistanceTo(o1.getX(),o1.getY());
            int dis2 = (int) me.getDistanceTo(o2.getX(),o2.getY());
            if (dis1 < dis2)
                return 1;
            else if (dis1 > dis2)
                return -1;
                            return 0;
            }
    }
	protected PriorityQueue<TWEntity> getNearbyAllSortedObjects(int sx, int sy, double threshold, Class<?> type) {
		//If we cannot find an object which we have seen recently, then we want
		//the one with maxTimestamp
		double maxTimestamp = 0;
		TWEntity o = null;
		double time = 0;
		PriorityQueue<TWEntity> ret = new PriorityQueue<TWEntity>(10, new ObjectComparator());
//		PriorityQueue<TWObject> ret = new PriorityQueue<TWEntity>(10,
//			new Comparator<TWObject>() {
//				public int compare(TWEntity o1, TWEntity o2) {
//					return (int) (((MyAgent)me).getDistanceTo(o1.getX(),o1.getY()) - ((MyAgent)me).getDistanceTo(o2.getX(),o2.getY()));
//				}});
		
//		TWObject ret = null;
		int x, y;
		for (Int2D offset : spiral) {
			x = offset.x + sx;
			y = offset.y + sy;

			if (me.getEnvironment().isInBounds(x, y) && 
					objects[x][y] != null &&
					!(objects[x][y].getO() instanceof TWFuelStation)) {
				o = (TWObject) objects[x][y].getO();//get mem object
				if (type.isInstance(o)) {//if it's not the type we're looking for do nothing

					time = objects[x][y].getT();//get time of memory

					if (this.getSimulationTime() - time <= threshold) {
						//if we found one satisfying time, then return
						//return o;
						ret.add(o);
					} else if (time > maxTimestamp) {
						//otherwise record the timestamp and the item in case
						//it's the most recent one we see
						//ret = o;
						ret.add(o);
						maxTimestamp = time;
					}
				}
			}
		}

		//this will either be null or the object of Class type which we have
		//seen most recently but longer ago than now-threshold.
		return ret;
	}

	
	/**
	 * Finds a nearby tile we have seen less than threshold timesteps ago
	 *
	 * @see TWAgentWorkingMemory#getNearbyObject(int, int, double, java.lang.Class)
	 */
	protected TWObject getNearbyObject(int sx, int sy, double threshold, Class<?> type) {

		//If we cannot find an object which we have seen recently, then we want
		//the one with maxTimestamp
		double maxTimestamp = 0;
		TWObject o = null;
		double time = 0;
		TWObject ret = null;
		int x, y;
		for (Int2D offset : spiral) {
			x = offset.x + sx;
			y = offset.y + sy;

			if (me.getEnvironment().isInBounds(x, y) 
					&& this.objects[x][y] != null
					&& !(objects[x][y].getO() instanceof TWFuelStation)) {
				o = (TWObject) this.objects[x][y].getO();//get mem object
				if (type.isInstance(o)
						) {//if it's not the type we're looking for do nothing

					time = this.objects[x][y].getT();//get time of memory

					if (this.getSimulationTime() - time <= threshold) {
						//if we found one satisfying time, then return
						return o;
					} else if (time > maxTimestamp) {
						//otherwise record the timestamp and the item in case
						//it's the most recent one we see
						ret = o;
						maxTimestamp = time;
					}
				}
			}
		}

		//this will either be null or the object of Class type which we have
		//seen most recently but longer ago than now-threshold.
		return ret;
	}
	
	/**
	 * Finds a nearby tile we have seen less than threshold timesteps ago
	 *
	 * @see TWAgentWorkingMemory#getNearbyObject(int, int, double, java.lang.Class)
	 */
	@Override
	public TWTile getNearbyTile(int x, int y, double threshold) {
		return (TWTile) this.getNearbyObject(x, y, threshold, TWTile.class);
	}

	/**
	 * Finds a nearby hole we have seen less than threshold timesteps ago
	 *
	 * @see TWAgentWorkingMemory#getNearbyObject(int, int, double, java.lang.Class)
	 */
	@Override
	public TWHole getNearbyHole(int x, int y, double threshold) {
		return (TWHole) this.getNearbyObject(x, y, threshold, TWHole.class);
	}
	
	
	public static void main(String[] args) {
		// TODO Auto-generated method stub

	}
	

}
