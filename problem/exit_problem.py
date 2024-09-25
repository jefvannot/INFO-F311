from lle import WorldState
from .problem import SearchProblem


class ExitProblem(SearchProblem[WorldState]):
    """
    A simple search problem where the agents must reach the exit **alive**.
    """

    def is_goal_state(self, state: WorldState) -> bool:
        # Check if all agents are alive
        if not all(state.agents_alive):
            return False
        
        # Check if all agents are at the exit positions
        for agent_position in state.agents_positions:
            if agent_position not in self.world.exit_pos:
                return False
        
        return True