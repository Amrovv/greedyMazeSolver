import uk.ac.warwick.dcs.maze.logic.IRobot;
import java.util.List;
import java.util.ArrayList;
import java.util.Random;
import java.util.Stack;
import java.util.Map;
import java.util.HashMap;
import java.util.Objects;


/**
* Explorer Class used to search for the target.
* Utilises three controllers, Explorer Backtracking and Ideal Path.
*/
public class SearchAlg{
    // Enum Mode for different states of the robot.
    enum Mode {
        START,
        EXPLORE,
        BACKTRACK,
        PERFECT_RUN
    }

    // Represents robot and target are on the same horizontal or vertical axis.
    private static final byte SAME_AXIS = 0;

    // Array of all available directions.
    private static final int[] DIRECTIONS = {IRobot.AHEAD, IRobot.RIGHT, IRobot.BEHIND, IRobot.LEFT};

    // Heading Stack object.
    Stack<Integer> headingStack = new Stack<>();

    // Ideal Path Stack.
    Stack<Junction> idealPath = new Stack<>();

    // Perfect Path Stack post optimization
    Stack<Junction> optimizedPath = new Stack<>();

    // Tracks the robot's current maze exploration state.
    private Mode controlMode = Mode.START;

    // Random object for selecting random direction.
    private final Random randomGen = new Random();

    /**
    * Executes when the maze is reset. Copies the ideal path into a temporary
    * working stack, optimizes it, and sets the control mode to PERFECT_RUN.
    */
   public void reset(){
         // Copy contents of idealPath into a working stack
        Stack<Junction> idealPathCopy = new Stack<>();
        idealPathCopy.addAll(idealPath);

        optimizedPath = optimizePath(idealPathCopy);
        controlMode = Mode.PERFECT_RUN;
    }

    /**
    * Optimizes a path by removing cycles. When a junction is revisited, all
    * junctions between the first and second occurrence are removed. This is
    * primarily used to simplify paths at crossroads, keeping only the final,
    * successful exit.
    *
    * @param pathStack the path to optimize
    * @return a path stack with cycles removed
    */
    public Stack<Junction> optimizePath(Stack<Junction> pathStack){

        Stack<Junction> optimizedStack = new Stack<>();

        // Tracks each junction and the index at which it was first added.
        Map<Junction, Integer> visitedJunctions = new HashMap<>();

        // Process all junctions, reversing order as we pop.
        while(pathStack.size() > 0) {

            Junction junction = pathStack.pop();

            // If we've seen this junction before, remove the cycle
            if(visitedJunctions.containsKey(junction)) {

               // Index of the first occurrence. Determines how far back we pop to remove the cycle.
                int index = visitedJunctions.get(junction);

               // Remove all junctions added after the first occurrence
                while(optimizedStack.size() > index + 1){
                    Junction removed = optimizedStack.pop();
                    visitedJunctions.remove(removed);
                }
            }
        // First occurrence. Push the junction onto the stack and add it to the map with its index.
        else{
            visitedJunctions.put(junction, optimizedStack.size());
            optimizedStack.push(junction);
        }
    }
    return optimizedStack;
}

    /**
    * Selects the appropriate control method based on the current control mode.
    * START mode:
    * - If the robot begins in a dead end, controlDeadEnd is used and the resulting heading
    *   is pushed onto idealPath.
    * - Otherwise, controlJunction is used for a corner with two exits.
    * - The resulting heading is always pushed onto headingStack for backtracking.
    * Otherwise appropriate control method is called.
    * Deals with new mazes being loaded by checking number of runs.
    * This method also handles newly loaded mazes by checking the run count
    * @param robot the robot whose direction is being determined
    */
    public void controlRobot(IRobot robot){

        // If a new maze is loaded after completing a perfect run, clear idealPath and
        // reset controlMode to START so a new optimal path can be computed.
        if(robot.getRuns() == 0 && controlMode == Mode.PERFECT_RUN){
            idealPath.clear();
            controlMode = Mode.START;
        }

        int direction;

        // Select appropriate controller based on the current control mode
        direction = switch(controlMode){
            case Mode.EXPLORE -> exploreControl(robot);
            case Mode.BACKTRACK -> backtrackControl(robot);
            // change mode an yield
            case Mode.START -> {

                    int nextDirection;

                    // If the robot starts in a dead end, use dead-end controller and
                    // store the leaving heading in idealPath.
                    if(nonWallExits(robot) == 1){
                        nextDirection = controlDeadEnd(robot);
                        idealPath.push(new Junction(robot.getLocation().x, robot.getLocation().y, nextHeading(robot, nextDirection)));
                    }
                    // Otherwise handle corner as a two-exit junction.
                    else nextDirection = controlJunction(robot);

                    // Push intial heading for backtracking
                    headingStack.push(nextHeading(robot, nextDirection));

                    controlMode = Mode.EXPLORE;

                    yield nextDirection;
                }
            case Mode.PERFECT_RUN -> optimalPathController(robot);
        };
        robot.face(direction);
    }

    /**
    * Controls the robot's movement during the optimal path run.
    * Determines movement based on the number of passage exits.
    * 1 passage: either a corridor or a dead end.
    *   - Corridor: controlCorridors is used.
    *   - Dead end: the next heading is popped from optimizedPath and converted to a relative direction.
    * 2 or 3 passages: at junctions/crossroads, the next heading is popped from optimizedPath and
    * converted to a relative direction.
    * @param robot the robot whose direction is being determined
    * @return the direction the robot should move
    */
    public int optimalPathController(IRobot robot){

            int direction;

            // Determine movement based on the number of available passages exits.
            direction = switch(passageExits(robot)){

                case 1 -> {
                        // If there are two non-wall exits, the robot is in a corridor
                        if(nonWallExits(robot) == 2){
                            yield controlCorridors(robot);
                        }
                        // Dead end: pop the next heading and convert it to a relative direction.
                        else yield toRelativeDirection(robot, optimizedPath.pop().getHeading());
                    }

                 // Junction or crossroads: pop the next heading and convert it to a relative direction.
                case 2, 3 -> toRelativeDirection(robot, optimizedPath.pop().getHeading());
                default -> -1;
            };

            return direction;
        }

    /**
    * Determines the appropriate controller to use during exploration based on the
    * number of non-wall exits around the robot. If the robot encounters a dead end,
    * the control mode switches to BACKTRACK.
    * @param robot the robot whose direction is being determined
    * @return the direction the robot should move
    */
    public int exploreControl(IRobot robot){

        int direction = 0 ;

        // Choose controller based on the number of non-wall exits.
        direction = switch(nonWallExits(robot)){

            // Dead end: switch to backtracking and handle dead end logic
            case 1 -> {controlMode = Mode.BACKTRACK; yield controlDeadEnd(robot);}
            // Corridor: continue forward following corridor logic.
            case 2 -> {
                        // Prevents the robot robot from incorrectly looping when returning to start cell.
                        if(robot.getLocation().x == 1 && robot.getLocation().y == 1){
                            controlMode = Mode.BACKTRACK;
                            yield IRobot.BEHIND;
                        }
                        yield controlCorridors(robot);
                    }
            case 3, 4 -> {
                        // If a junction or crossroads has no passage exits, switch to backtracking and turn around.
                        if(passageExits(robot) < 1){
                            controlMode = Mode.BACKTRACK;
                            yield IRobot.BEHIND;
                        }

                        // Otherwise, push the current heading and use junction controller to determine direction.
                        headingStack.push(robot.getHeading());

                        yield controlJunction(robot);
                    }
            default -> -1;
        };

        return direction;
    }


    /**
    * Controls the robot's movement during backtracking mode.
    * Corridor contains a special case:
    *   - If the robot begins at a corner with two exits and chooses a corridor, that leads to a dead end,
    *     it must backtrack to the start cell, remove the incorrect heading from idealPath, and
    *     switch back to explorer mode to take the other corridor.
    * At junctions or crossroads with available passage exits, the robot switches back
    * to explore mode and calls the junction controller.
    * At junctions or crossroads with no available passage exits, the robot retrieves its arrival heading
    * from headingStack and reverses direction. The unsuccesful heading is removed from idealPath.
    * @param robot the robot whose direction is being determined
    * @return the direction the robot should move
   */
    public int backtrackControl(IRobot robot){

        int direction = 0;

       // Choose action based on the number of non-wall exits.
        direction = switch(nonWallExits(robot)){

            // Dead end: Handle normally
            case 1 -> controlDeadEnd(robot);

            // Corridor, turn or corner.
            case 2 -> {
                        // Start-of-run correction: robot chose a corridor that led to a dead end.
                        if(passageExits(robot) > 0){
                            controlMode = Mode.EXPLORE;

                            // Remove unsuccesful heading, switch to explore mode and re-handle this corner.
                            idealPath.pop();
                            controlMode = Mode.EXPLORE;
                            yield controlJunction(robot);
                        }
                        yield controlCorridors(robot);
                    }
            case 3, 4 -> {
                        // If passages exists, switch back to explorer mode.
                        if(passageExits(robot) > 0){

                            // Remove unsuccesful heading, switch to explore mode and re-handle this junction.
                            idealPath.pop();
                            controlMode = Mode.EXPLORE;
                            yield controlJunction(robot);
                        }
                        // If no passages, retrieve arrival heading and turn opposite to backtrack.
                        robot.setHeading(headingStack.pop());

                        // Remove the unsuccessful heading.
                        idealPath.pop();
                        // Reverse
                        yield IRobot.BEHIND;
                }
            default -> -1;
        };

        return direction;
    }

    /**
    * Counts the number of non-wall cells surrounding the robot.
    * @param robot robot object.
    * @return number of non-wall cells surrounding robot.
    */
    private int nonWallExits(IRobot robot){

        int numNonWalls = 0;

        // Count all adjacent cells that are not walls.
        for(int direction:DIRECTIONS){
            if(robot.look(direction) != IRobot.WALL){
                numNonWalls++;
            }
        }
        return numNonWalls;
    }

    /**
    * Counts the number of passage cells surrounding the robot.
    * @param robot robot object.
    * @return number of passage cells surrounding robot.
    */
    private int passageExits(IRobot robot){

        int numPassages = 0;

        // Count all passage cells adjacent to the robot.
        for(int direction:DIRECTIONS){
            if(robot.look(direction) == IRobot.PASSAGE){
                numPassages ++;
            }
        }
        return numPassages;
    }

   /**
    * Controls the robot's movement at a dead end.
    * - In START mode, returns the first available non-wall direction.
    * - In all other modes, returns IRobot.BEHIND to backtrack.
    * @param robot the robot whose direction is being determined
    * @return a non-wall direction in START mode, otherwise IRobot.BEHIND
    */
    private int controlDeadEnd(IRobot robot){

        // When in START mode, choose the first non-wall direction
        if(controlMode == Mode.START){
            for(int direction:DIRECTIONS){
                if(robot.look(direction) != IRobot.WALL){
                    return direction;
                }
            }
        }
        // Otherwise, reverse direction to backtrack.
        return IRobot.BEHIND;
    }


    /**
    * Control robot's movement in a corridor.
    * Returns the first non-wall direction that is not IRobot.BEHIND.
    * Only one such direction will exist in a corridor.
    * @param robot the robot object whose direction is to be set.
    * @return IRobot.AHEAD, IRobot.LEFT or IRobot.RIGHT, depending which does not lead into a wall.
    */
    private int controlCorridors(IRobot robot){

         // Select the first direction that is not behind and not a wall
        for(int direction:DIRECTIONS){
            if(direction != IRobot.BEHIND && robot.look(direction) != IRobot.WALL){
                return direction;
            }
        }
        return -1;
    }

    /**
    * Computes the robot's absolute heading after making a move.
    * Given the robot's current heading and the relative move direction,
    * this method calculates what the new absolute heading will be.
    * @param robot the robot whose current heading is used
    * @param moveDirection the relative direction the robot will turn to
    * @return the resulting absolute heading
    */
    private int nextHeading(IRobot robot, int moveDirection){

        // Convert relative move direction into a turn offset.
        int turn = moveDirection - IRobot.AHEAD;

        // Convert the robot's current heading into an offset from IRobot.NORTH.
        int headingOffset = robot.getHeading() - IRobot.NORTH;

        // Combine heading and turn offsets, wrap with modulo, and find new absolute heading.
        return IRobot.NORTH + ((headingOffset + turn) % 4);
    }

    /**
    * Converts an absolute heading into a direction relative to the robot's current heading.
    * @param robot the robot whose current heading is used as reference
    * @param heading the absolute heading to convert
    * @return one of IRobot.AHEAD, IRobot.RIGHT, IRobot.BEHIND, or IRobot.LEFT
    */
    private int toRelativeDirection(IRobot robot, int heading){

        // Calculate how far the target heading is from the robot's current heading,
        // ensure a positive value, and wrap it into the 0–3 range.
        return IRobot.AHEAD + ((heading - robot.getHeading() + 4) % 4);
    }

    /**
    * Controls the robot's movement at junctions and crossroads during EXPLORE mode.
    * Determines the target's position relative to the robot using the
    * isTargetEast and isTargetNorth methods, producing two directional values:
    * - xDirection: EAST or WEST if the target lies horizontally to either side,
    *               or SAME_AXIS if on the same column.
    * - yDirection: NORTH or SOUTH if the target lies vertically above or below,
    *               or SAME_AXIS if on the same row.
    * Each absolute direction is converted to a relative direction using toRelativeDirection.
    * If either xDirection or yDirection is not SAME_AXIS and leads to a passage cell
    * it is added to the list of passageDirections.
    * In neither meets the conditons, a random passage exit is chosen.
    * In all cases, the chosen direction results in a new heading, new Junction object pushed into stack.
    * @param robot the robot whose direction is being determined
    * @return a passage direction, potentially biased toward the target's direction
    */
    private int controlJunction(IRobot robot){

        // Determine horizontal location of target relative to robot.
        // Convert absolute EAST/WEST to relative direction
        int xDirection = switch(isTargetEast(robot)){
            case 1 -> xDirection = toRelativeDirection(robot, IRobot.EAST);
            case 0 -> SAME_AXIS;
            case -1 -> xDirection = toRelativeDirection(robot, IRobot.WEST);
            default -> -1;
        };

        // Determine vertical location of target relative to robot.
        // Convert absolute NORTH/SOUTH to relative direction
        int yDirection = switch(isTargetNorth(robot)){
            case 1 -> toRelativeDirection(robot, IRobot.NORTH);
            case 0 -> SAME_AXIS;
            case -1 -> toRelativeDirection(robot, IRobot.SOUTH);
            default -> -1;
        };

         // List of all possible passage directions.
        List<Integer> passageDirections = new ArrayList<>();

        // Add the horizontal direction if it's not SAME_AXIS and leads to a passage.
        if(xDirection != SAME_AXIS && robot.look(xDirection) == IRobot.PASSAGE){
            passageDirections.add(xDirection);
        }

        // Add the vertical direction if it's not SAME_AXIS and leads to a passage
        if(yDirection != SAME_AXIS && robot.look(yDirection) == IRobot.PASSAGE){
            passageDirections.add(yDirection);
        }

        // Otherwise, collect all possible passage exits.
        if(passageDirections.isEmpty()){
            for(int direction:DIRECTIONS){
                if(robot.look(direction) == IRobot.PASSAGE){
                        passageDirections.add(direction);
                }
            }
        }
        // Choose a random passage direction from the list
        int direction = passageDirections.get(randomGen.nextInt(passageDirections.size()));

       // Record the Junction with the resulting heading after the robot takes this direction.
        idealPath.push(new Junction(robot.getLocation().x, robot.getLocation().y, nextHeading(robot, direction)));


        return direction;
    }


    /**
    * Determines whether the robot is north, south, or level relative to the target.
    *
    * @param robot the robot being checked.
    * @return 1 if the target is north, -1 if south, 0 if same row.
    */

    private byte isTargetNorth(IRobot robot) {

        int robotY = robot.getLocation().y;
        int targetY = robot.getTargetLocation().y;

        if(targetY < robotY){  //Target is North.
            return 1;
        }
        else if (targetY > robotY){  // Target is South.
            return -1;
        }
        else return 0;   // Same row.
    }

    /**
    * Determines whether the robot is east, west, or level relative to the target.
    *
    * @param robot the robot being checked.
    * @return 1 if the target is east, -1 if west, 0 if same column.
    */
    private byte isTargetEast(IRobot robot){
        int robotX = robot.getLocation().x;
        int targetX = robot.getTargetLocation().x;

        if(targetX > robotX){   // Target is East.
            return 1;
        }
        else if (targetX < robotX){   // Target is West.
            return -1;
        }
        else return 0;     // Same column.
    }
}

/**
 * Represents a junction/crossroads cell in the maze.
 * equals() and hashCode() are based only on the junction's coordinates.
 */
class Junction{

    int juncX;
    int juncY;
    int heading;

    public Junction(int juncX, int juncY, int heading){
        this.juncX = juncX;
        this.juncY = juncY;
        this.heading = heading;
    }

    public int getHeading(){
        return heading;
    }

    // Equals compares only coordinates to detect revisiting the same junction location.
    public boolean equals(Object o) {
        if(!(o instanceof Junction)){
            return false;}
        Junction junction = (Junction) o;
        return juncX == junction.juncX && juncY == junction.juncY;
    }

    // Hash code matches equals: based solely on coordinates.
    public int hashCode() {
        return Objects.hash(juncX, juncY);
    }
}
