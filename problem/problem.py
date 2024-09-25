from abc import ABC, abstractmethod
from typing import Generic, TypeVar
from lle import World, Action, WorldState


S = TypeVar("S", bound=WorldState)


class SearchProblem(ABC, Generic[S]):
    """
    A Search Problem is a problem that can be solved by a search algorithm.
    The generic parameter S is the type of the problem state.
    """

    def __init__(self, world: World):
        self.world = world
        world.reset()
        self.initial_state = world.get_state()

    @abstractmethod
    def is_goal_state(self, problem_state: S) -> bool:
        """Whether the given state is the goal state"""


    def get_successors(self, state: S) -> list[tuple[WorldState, Action]]:
        """
        Returns  all possible states that can be reached from the given world state.

        Note that if an agent dies, the game is over and there is no successor to that state.
        """
        successors = []
        self.world.set_state(state)
        for actions in self.world.available_actions():
            for action in actions:
                events = self.world.step(action)
                new_state = self.world.get_state()
                if all(agent_alive for agent_alive in new_state.agents_alive):
                   successors.append((new_state, action))
                self.world.set_state(state)
        return successors



    def heuristic(self, problem_state: S) -> float:
        ### Manhattan
        total_distance = 0
        for agent_pos in problem_state.agents_positions:
            min_distance = float('inf')
            for gem_pos, collected in zip(self.world.gems.keys(), problem_state.gems_collected):
                if not collected:
                    distance = abs(agent_pos[0] - gem_pos[0]) + abs(agent_pos[1] - gem_pos[1])
                    if distance < min_distance:
                        min_distance = distance
            total_distance += min_distance
        return total_distance
