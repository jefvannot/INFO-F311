from lle import WorldState
from .problem import SearchProblem


class GemProblem(SearchProblem[WorldState]):

    def is_goal_state(self, state: WorldState) -> bool:
        if not all(state.agents_alive):
            return False

        for agent_position in state.agents_positions:
            if agent_position not in self.world.exit_pos:
                return False

        if not len(state.gems_collected):
            return True

        return all(state.gems_collected)
        # return (not len(state.gems_collected)) or all(state.gems_collected)

