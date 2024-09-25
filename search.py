from dataclasses import dataclass
from typing import Generic, Optional, TypeVar

from lle import Action, WorldState

from priority_queue import PriorityQueue
from problem import SearchProblem

from collections import deque 
import heapq

import cv2

S = TypeVar("S", bound=WorldState)

@dataclass
class Solution(Generic[S]):
    actions: list[Action]
    states: list[S]

    @property
    def n_steps(self) -> int:
        return len(self.actions)

    @staticmethod
    def from_node(node: "SearchNode") -> "Solution[S]":
        actions = []
        states = []
        while node.parent is not None:
            actions.append(node.prev_action)
            states.append(node.state)
            node = node.parent
        actions.reverse()
        return Solution(actions, states)


@dataclass
class SearchNode:
    state: WorldState
    parent: Optional["SearchNode"]
    prev_action: Optional[Action]
    cost: float = 0.0

    def __hash__(self) -> int:
        return hash((self.state, self.cost))

    def __eq__(self, other: object) -> bool:
        if not isinstance(other, SearchNode):
            return NotImplemented
        return self.state == other.state and self.cost == other.cost

    def __lt__(self, other):
        return self.cost < other.cost


def dfs(problem: SearchProblem) -> Optional[Solution]:
    stack = [SearchNode(problem.initial_state, None, None)]
    explored = set()
    # in_queue = set([problem.initial_state])

    while stack:
        node = stack.pop()
        # in_queue.remove(node.state)
        if problem.is_goal_state(node.state):
            ### to fix the missing world.reset() line in check_corner_problem()
            problem.world.reset()
            print(len(explored))
            return Solution.from_node(node)
        explored.add(node.state)
        for successor, action in problem.get_successors(node.state):
            # if successor not in explored and successor not in in_queue:
            if successor not in explored:
                stack.append(SearchNode(successor, node, action))
                # in_queue.add(successor)
    return None

def bfs(problem: SearchProblem) -> Optional[Solution]:
    queue = deque([SearchNode(problem.initial_state, None, None)])
    explored = set()
    in_queue = set([problem.initial_state])

    while queue:
        node = queue.popleft()
        in_queue.remove(node.state)
        if problem.is_goal_state(node.state):
            ### to fix the missing world.reset() line in check_corner_problem()
            problem.world.reset()
            print(len(explored))
            return Solution.from_node(node)
        explored.add(node.state)
        for successor, action in problem.get_successors(node.state):
            if successor not in explored and successor not in in_queue:
                queue.append(SearchNode(successor, node, action))
                in_queue.add(successor)
    return None


def astar(problem: SearchProblem) -> Optional[Solution]:
    frontier = []
    heapq.heappush(frontier, (0, SearchNode(problem.initial_state, None, None)))
    explored = set()
    cost_so_far = {problem.initial_state: 0}
    in_frontier = {problem.initial_state: 0}

    while frontier:
        _, node = heapq.heappop(frontier)
        current_state = node.state
        if problem.is_goal_state(current_state):
            problem.world.reset()
            print(len(explored))
            return Solution.from_node(node)
        explored.add(current_state)
        for successor, action in problem.get_successors(current_state):
            new_cost = node.cost + 1
            if successor not in explored and (successor not in in_frontier or new_cost < cost_so_far[successor]):
                cost_so_far[successor] = new_cost
                heuristic_cost = new_cost + problem.heuristic(successor)
                heapq.heappush(frontier, (heuristic_cost, SearchNode(successor, node, action, new_cost)))
                in_frontier[successor] = heuristic_cost
    return None


