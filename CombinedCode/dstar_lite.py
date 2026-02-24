# ==========================================================
# dstar_lite.py
# Orientation-agnostic D* Lite implementation
# Compatible with arena.py contract
# ==========================================================

import heapq
import math


# ==========================================================
# COST MODEL
# ==========================================================

class CostModel:
    """
    Customizable cost model.
    Modify node_cost or edge_cost later without touching planner.
    """

    def __init__(self):
        self._node_costs = {}

    def set_node_cost(self, step, cost):
        self._node_costs[step] = cost

    def node_cost(self, step):
        return self._node_costs.get(step, 0)

    def edge_cost(self, a, b):
        return 1


# ==========================================================
# PRIORITY QUEUE
# ==========================================================

class PriorityQueue:
    def __init__(self):
        self.heap = []
        self.entry_finder = {}

    def put(self, item, priority):
        entry = (priority, item)
        self.entry_finder[item] = entry
        heapq.heappush(self.heap, entry)

    def remove(self, item):
        entry = self.entry_finder.pop(item, None)
        return entry

    def pop(self):
        while self.heap:
            priority, item = heapq.heappop(self.heap)
            if item in self.entry_finder:
                del self.entry_finder[item]
                return item, priority
        raise KeyError("pop from empty priority queue")

    def top_key(self):
        while self.heap:
            priority, item = self.heap[0]
            if item in self.entry_finder:
                return priority
            heapq.heappop(self.heap)
        return (math.inf, math.inf)

    def contains(self, item):
        return item in self.entry_finder


# ==========================================================
# D* LITE
# ==========================================================

class DStarLite:

    def __init__(self, graph, start, goals, cost_model):

        self.graph = graph
        self.start = start
        self.goals = goals
        self.cost_model = cost_model

        self.g = {s: math.inf for s in graph}
        self.rhs = {s: math.inf for s in graph}

        self.km = 0

        self.U = PriorityQueue()

        for goal in goals:
            self.rhs[goal] = 0
            self.U.put(goal, self.calculate_key(goal))

    # ------------------------------------------------------

    def heuristic(self, a, b):
        ar = self.graph[a]["row"]
        ac = self.graph[a]["col"]

        br = self.graph[b]["row"]
        bc = self.graph[b]["col"]

        return abs(ar - br) + abs(ac - bc)

    # ------------------------------------------------------

    def calculate_key(self, s):
        g_rhs = min(self.g[s], self.rhs[s])
        return (
            g_rhs + self.heuristic(self.start, s) + self.km,
            g_rhs
        )

    # ------------------------------------------------------

    def successors(self, s):
        return [
            nb for nb in self.graph[s]["neighbors"].values()
            if nb is not None
        ]

    def predecessors(self, s):
        preds = []
        for node, data in self.graph.items():
            for nb in data["neighbors"].values():
                if nb == s:
                    preds.append(node)
        return preds

    # ------------------------------------------------------

    def cost(self, a, b):
        return (
            self.cost_model.edge_cost(a, b) +
            self.cost_model.node_cost(b)
        )

    # ------------------------------------------------------

    def update_vertex(self, u):

        if u not in self.goals:
            min_rhs = math.inf
            for s in self.successors(u):
                val = self.cost(u, s) + self.g[s]
                if val < min_rhs:
                    min_rhs = val
            self.rhs[u] = min_rhs

        if self.U.contains(u):
            self.U.remove(u)

        if self.g[u] != self.rhs[u]:
            self.U.put(u, self.calculate_key(u))

    # ------------------------------------------------------

    def compute_shortest_path(self):

        while (
            self.U.top_key() < self.calculate_key(self.start)
            or self.rhs[self.start] != self.g[self.start]
        ):

            u, k_old = self.U.pop()
            k_new = self.calculate_key(u)

            if k_old < k_new:
                self.U.put(u, k_new)

            elif self.g[u] > self.rhs[u]:
                self.g[u] = self.rhs[u]
                for p in self.predecessors(u):
                    self.update_vertex(p)

            else:
                self.g[u] = math.inf
                self.update_vertex(u)
                for p in self.predecessors(u):
                    self.update_vertex(p)

    # ------------------------------------------------------

    def get_next_step(self):
        """
        Returns the best next step from current start.
        """

        min_cost = math.inf
        next_step = None

        for s in self.successors(self.start):
            cost = self.cost(self.start, s) + self.g[s]
            if cost < min_cost:
                min_cost = cost
                next_step = s

        return next_step

    # ------------------------------------------------------

    def move_start(self, new_start):
        self.start = new_start

    # ------------------------------------------------------

    def notify_cost_change(self, step):
        """
        Call when obstacle cost changes.
        """
        self.update_vertex(step)
        for p in self.predecessors(step):
            self.update_vertex(p)

    def clear_node_cost(self, step):
        if step in self._node_costs:
            del self._node_costs[step]