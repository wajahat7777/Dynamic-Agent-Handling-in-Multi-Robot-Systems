#!/usr/bin/env python
# coding: utf-8

# In[3]:


#Wajahat Ullah Khan
#Roll NO : 22i-0776
#Section : G


# In[2]:


from typing import List, Tuple, Dict, Set
from queue import PriorityQueue
import random
from dataclasses import dataclass
from enum import Enum
import heapq
import re

class CellType(Enum):
    FREE = 0
    OBSTACLE = 1
    DYNAMIC = 2
    GOAL = 3

@dataclass(order=True)
class Position:
    x: int
    y: int
    
    def __hash__(self):
        return hash((self.x, self.y))
    
    def __eq__(self, other):
        return self.x == other.x and self.y == other.y

class DynamicAgent:
    def __init__(self, path: List[Tuple[int, int]]):
        self.base_path = [Position(x, y) for x, y in path]
        self.full_path = self._create_full_path()
        
    def _create_full_path(self) -> List[Position]:
        # Create the full path including the reverse journey
        forward_path = self.base_path
        reverse_path = self.base_path[::-1]  # Include all points in reverse
        return forward_path + reverse_path[1:]  # Skip the first point to avoid duplicate
        
    def get_position_at_time(self, time: int) -> Position:
        if not self.full_path:
            return None
        # Calculate the effective position in the repeating pattern
        cycle_length = len(self.full_path)
        effective_time = time % cycle_length
        return self.full_path[effective_time]

class Robot:
    def __init__(self, start: Position, goal: Position, id: int):
        self.start = start
        self.goal = goal
        self.id = id
        self.path = []
        self.total_time = 0

class Grid:
    def __init__(self, width: int, height: int):
        self.width = width
        self.height = height
        self.grid = [[CellType.FREE for _ in range(width)] for _ in range(height)]
        self.dynamic_agents = []

    def set_obstacle(self, pos: Position):
        self.grid[pos.x][pos.y] = CellType.OBSTACLE

    def is_valid_position(self, pos: Position) -> bool:
        return (
            0 <= pos.x < self.width
            and 0 <= pos.y < self.height
            and self.grid[pos.x][pos.y] != CellType.OBSTACLE
        )

    def add_dynamic_agent(self, agent: DynamicAgent):
        self.dynamic_agents.append(agent)

    def is_position_safe(self, pos: Position, time: int) -> bool:
        if not self.is_valid_position(pos):
            return False

        # Check for dynamic agents
        for agent in self.dynamic_agents:
            agent_pos = agent.get_position_at_time(time)
            if agent_pos and agent_pos == pos:
                return False
        return True

@dataclass(order=True)
class PrioritizedItem:
    priority: float
    time: int = 0
    position: Position = None

class PathFinder:
    def __init__(self, grid: Grid):
        self.grid = grid
        self.directions = [
            Position(0, 1),   # right
            Position(1, 0),   # down
            Position(-1, 0),  # up
            Position(0, -1)   # left
        ]
    
    def heuristic(self, pos: Position, goal: Position) -> float:
        return abs(pos.x - goal.x) + abs(pos.y - goal.y)
    
    def get_neighbors(self, pos: Position, time: int) -> List[Position]:
        neighbors = []
        for direction in self.directions:
            new_pos = Position(pos.x + direction.x, pos.y + direction.y)
            if self.grid.is_position_safe(new_pos, time + 1):
                neighbors.append(new_pos)
        return neighbors
    
    def find_path(self, robot: Robot, other_robot_positions: Dict[int, Set[Position]]) -> bool:
        pq = []
        start_item = PrioritizedItem(
            priority=self.heuristic(robot.start, robot.goal),
            time=0,
            position=robot.start
        )
        heapq.heappush(pq, start_item)
        
        came_from = {}
        g_score = {robot.start: 0}
        f_score = {robot.start: self.heuristic(robot.start, robot.goal)}
        
        while pq:
            current = heapq.heappop(pq)
            current_pos = current.position
            current_time = current.time
            
            if current_pos == robot.goal:
                # Reconstruct path
                path = []
                while current_pos in came_from:
                    path.append(current_pos)
                    current_pos = came_from[current_pos]
                path.append(robot.start)
                path.reverse()
                robot.path = path
                robot.total_time = current_time
                return True
            
            for next_pos in self.get_neighbors(current_pos, current_time):
                # Check for collisions with other robots
                if current_time + 1 in other_robot_positions and next_pos in other_robot_positions[current_time + 1]:
                    print(f"Collision detected for Robot {robot.id} at {next_pos} at time {current_time + 1}. Retrying...")
                    continue  # Skip this position and try a different direction
                    
                tentative_g = g_score[current_pos] + 1
                
                if next_pos not in g_score or tentative_g < g_score[next_pos]:
                    came_from[next_pos] = current_pos
                    g_score[next_pos] = tentative_g
                    f = tentative_g + self.heuristic(next_pos, robot.goal)
                    f_score[next_pos] = f
                    next_item = PrioritizedItem(
                        priority=f,
                        time=current_time + 1,
                        position=next_pos
                    )
                    heapq.heappush(pq, next_item)
        
        print(f"No valid path found for Robot {robot.id}.")
        return False
    
def solve_multi_robot_pathfinding(
    grid_size: Tuple[int, int],
    obstacles: List[Tuple[int, int]],
    robot_starts: List[Tuple[int, int]],
    robot_goals: List[Tuple[int, int]],
    dynamic_agents_paths: List[List[Tuple[int, int]]]
) -> List[Tuple[List[Tuple[int, int]], int]]:
    
    # Initialize grid
    grid = Grid(grid_size[0], grid_size[1])
    
    # Add obstacles
    for obs in obstacles:
        grid.set_obstacle(Position(obs[0], obs[1]))
    
    # Add dynamic agents (now only need paths, no times)
    for agent_path in dynamic_agents_paths:
        grid.add_dynamic_agent(DynamicAgent(agent_path))
    
    # Create robots
    robots = [
        Robot(Position(start[0], start[1]), Position(goal[0], goal[1]), i)
        for i, (start, goal) in enumerate(zip(robot_starts, robot_goals))
    ]
    
    # Initialize pathfinder
    pathfinder = PathFinder(grid)
    
    # Find paths for all robots
    other_robot_positions: Dict[int, Set[Position]] = {}
    
    for robot in robots:
        success = pathfinder.find_path(robot, other_robot_positions)
        
        if success:
            # Update other robot positions for collision avoidance
            for t, pos in enumerate(robot.path):
                if t not in other_robot_positions:
                    other_robot_positions[t] = set()
                other_robot_positions[t].add(pos)
    
    # Convert paths to the required output format
    result = []
    for robot in robots:
        path_tuples = [(pos.x, pos.y) for pos in robot.path]
        result.append((path_tuples, robot.total_time))
    
    return result

def parse_agent_file(filename: str) -> List[List[Tuple[int, int]]]:
    """Parse agent file containing paths and times for dynamic agents"""
    agent_paths = []
    
    with open(filename, 'r') as f:
        content = f.read()
        
    # Extract path coordinates using regex
    pattern = r'\[\((.*?)\)\]'
    for line in content.split('\n'):
        if not line.strip():
            continue
            
        path_match = re.search(pattern, line)
        if path_match:
            path_str = path_match.group(1)
            # Extract coordinates and convert to list of tuples
            coords = re.findall(r'\((\d+),\s*(\d+)\)', path_str)
            path = [(int(x), int(y)) for x, y in coords]
            agent_paths.append(path)
    
    return agent_paths

def parse_grid_file(filename: str) -> Tuple[List[List[bool]], int]:
    """Parse grid file containing the map layout"""
    grid = []
    size = 0
    
    with open(filename, 'r') as f:
        # First line contains grid size
        size = int(f.readline().strip())
        
        # Read exactly 'size' number of rows
        for _ in range(size):
            line = f.readline().strip() if not f.readline() == '' else ''
            cells = line.split() if line else []
            # Truncate or pad the row to ensure it has exactly 'size' elements
            row = [False] * size  # Default to free cells
            for i in range(min(len(cells), size)):
                row[i] = (cells[i] == 'X')
            grid.append(row)
    
    return grid, size

def parse_robots_file(filename: str) -> Tuple[List[Tuple[int, int]], List[Tuple[int, int]]]:
    """Parse robots file containing start and goal positions"""
    starts = []
    goals = []
    
    with open(filename, 'r') as f:
        for line in f:
            if not line.strip():
                continue
                
            # Extract start and end coordinates
            matches = re.findall(r'\((\d+),\s*(\d+)\)', line)
            if len(matches) == 2:
                start, goal = matches
                starts.append((int(start[0]), int(start[1])))
                goals.append((int(goal[0]), int(goal[1])))
    
    return starts, goals

def load_problem_instance(data_file: str, agent_file: str, robots_file: str) -> Dict:
    """Load all problem data from files and return as a dictionary"""
    # Parse grid
    grid, grid_size = parse_grid_file(data_file)
    
    # Parse dynamic agents
    dynamic_agents = parse_agent_file(agent_file)
    
    # Parse robot configurations
    robot_starts, robot_goals = parse_robots_file(robots_file)
    
    # Convert grid to list of obstacle positions
    obstacles = []
    for i in range(len(grid)):
        for j in range(len(grid[i])):
            if grid[i][j]:  # If True, it's an obstacle
                obstacles.append((i, j))
    
    return {
        'grid_size': (grid_size, grid_size),
        'obstacles': obstacles,
        'dynamic_agents_paths': dynamic_agents,
        'robot_starts': robot_starts,
        'robot_goals': robot_goals
    }

def visualize_grid(grid, robot_starts, robot_goals):
    for i in range(grid.height):
        for j in range(grid.width):
            pos = Position(i, j)
            if pos in [Position(*start) for start in robot_starts]:
                print("S", end=" ")  # Start position
            elif pos in [Position(*goal) for goal in robot_goals]:
                print("G", end=" ")  # Goal position
            elif grid.grid[i][j] == CellType.OBSTACLE:  # FIXED HERE
                print("X", end=" ")  # Obstacle
            else:
                print(".", end=" ")  # Free cell
        print()


def visualize_path_and_time(paths_and_times: List[Tuple[List[Tuple[int, int]], int]]):
    print("\n=== Robot Paths and Execution Times ===")
    for robot_id, (path, total_time) in enumerate(paths_and_times):
        print(f"\nRobot {robot_id}:")
        print(f"Total time steps: {total_time}")
        print("Path coordinates (x, y):")
        for step, (x, y) in enumerate(path):
            print(f"t={step}: ({x}, {y})")

if __name__ == "__main__":
    # Load problem instance
    problem_data = load_problem_instance(
        r'C:\Users\wajah\Desktop\Input Files\Data\data0.txt',
        r'C:\Users\wajah\Desktop\Input Files\Data\Agent0.txt',
        r'C:\Users\wajah\Desktop\Input Files\Data\Robots0.txt'
    )

    # Initialize grid
    grid = Grid(problem_data['grid_size'][0], problem_data['grid_size'][1])

    # Add obstacles to the grid
    for obs in problem_data['obstacles']:
        grid.set_obstacle(Position(obs[0], obs[1]))

    # Add dynamic agents to the grid
    for agent_path in problem_data['dynamic_agents_paths']:
        grid.add_dynamic_agent(DynamicAgent(agent_path))

    # Solve the pathfinding problem
    paths_and_times = solve_multi_robot_pathfinding(
        problem_data['grid_size'],
        problem_data['obstacles'],
        problem_data['robot_starts'],
        problem_data['robot_goals'],
        problem_data['dynamic_agents_paths']
    )

    # Visualize the results
    visualize_path_and_time(paths_and_times)


# In[ ]:





# In[ ]:




