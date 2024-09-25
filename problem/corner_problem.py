from lle import WorldState, World, Position
from .problem import SearchProblem
from typing import Tuple, List, Optional


# class CornerState(WorldState):
#     def __init__(self, agents_positions: List[Position], gems_collected: List[bool], agents_alive: Optional[List[bool]], visited_corners: List[bool]):
#         super().__init__(agents_positions, gems_collected, agents_alive)
#         self.visited_corners = visited_corners

class CornerState(WorldState):
    def __new__(cls, agents_positions: List[Position], gems_collected: List[bool], agents_alive: Optional[List[bool]] = None, *args, **kwargs):
        return super(CornerState, cls).__new__(cls, agents_positions, gems_collected, agents_alive)

    def __init__(self, agents_positions: List[Position], gems_collected: List[bool], agents_alive: Optional[List[bool]] = None, visited_corners: Optional[Tuple[bool, bool, bool, bool]] = None):
        super().__init__(agents_positions, gems_collected, agents_alive)
        self.visited_corners = visited_corners or ([(False, False, False, False)] * len(agents_positions))
#
#
    def __hash__(self) -> int:
        return hash((tuple(self.agents_positions), tuple(self.gems_collected), tuple(self.agents_alive), tuple(self.visited_corners)))

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, CornerState):
            return NotImplemented
        return (self.agents_positions == other.agents_positions and
                self.gems_collected == other.gems_collected and
                self.agents_alive == other.agents_alive and
                self.visited_corners == other.visited_corners)

    def __str__(self) -> str:
        return f"CornerState(agents_positions={self.agents_positions}, gems_collected={self.gems_collected}, agents_alive={self.agents_alive}, visited_corners={self.visited_corners})"



class CornerProblem(SearchProblem[CornerState]):

    def __init__(self, world: World):
        super().__init__(world)
        self.corners = [(0, 0), (0, world.width - 1), (world.height - 1, 0), (world.height - 1, world.width - 1)]
        self.initial_state = CornerState(world.agents_positions, [False] * world.n_gems, [True] * world.n_agents, [False] * 4)

    def is_goal_state(self, state: CornerState) -> bool:
        result = all(state.visited_corners) and all(agent_position in self.world.exit_pos for agent_position in state.agents_positions)
        self.world.reset()
        return result

    def get_successors(self, state: CornerState):
        successors = []
        for world_state, actions in super().get_successors(state):
            visited_corners = list(state.visited_corners)
            for i, agent_position in enumerate(world_state.agents_positions):
                if agent_position in self.corners:
                    visited_corners[self.corners.index(agent_position)] = True
            next_state = CornerState(world_state.agents_positions, world_state.gems_collected, world_state.agents_alive, tuple(visited_corners))
            successors.append((next_state, actions))
        return successors


    # def heuristic(problem_state: WorldState, corners: List[Tuple[int, int]]) -> float:
    #     unvisited_corners = [corner for corner, visited in zip(corners, problem_state.gems_collected) if not visited]
    #     if not unvisited_corners:
    #         return 0

    #     # Calculate the Manhattan distance to the nearest unvisited corner
    #     agent_pos = problem_state.agents_positions[0]  # Assuming a single agent
    #     min_distance = float('inf')
    #     for corner in unvisited_corners:
    #         distance = abs(agent_pos[0] - corner[0]) + abs(agent_pos[1] - corner[1])
    #         if distance < min_distance:
    #             min_distance = distance

    #     # Approximate the MST of the unvisited corners
    #     mst_cost = approximate_mst_cost(unvisited_corners)
    #     return min_distance + mst_cost

    # def approximate_mst_cost(points: List[Tuple[int, int]]) -> float:
    #     if not points:
    #         return 0

    #     mst_cost = 0
    #     visited = set()
    #     min_heap = [(0, points[0])]

    #     while min_heap:
    #         cost, point = heapq.heappop(min_heap)
    #         if point in visited:
    #             continue
    #         visited.add(point)
    #         mst_cost += cost

    #         for next_point in points:
    #             if next_point not in visited:
    #                 next_cost = abs(point[0] - next_point[0]) + abs(point[1] - next_point[1])
    #                 heapq.heappush(min_heap, (next_cost, next_point))

    #     return mst_cost



