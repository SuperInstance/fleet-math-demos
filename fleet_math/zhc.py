"""Zero Holonomy Consensus (ZHC).

Models a directed constraint graph where edges carry weights representing
obstacle-phase differences. A configuration has zero holonomy iff the
product of weights around every cycle equals 1 (or equivalently, the sum
of log-weights around every cycle equals 0).
"""

import math


class ConstraintGraph:
    """Weighted undirected graph for holonomy consensus checking.

    Edge weights are stored as positive floats; the holonomy around a cycle
    is the product of weights along that cycle.
    """

    def __init__(self):
        # Adjacency dict: {node: {neighbor: weight}}
        self._adj = {}
        self._weights = {}  # {(u,v) sorted tuple: weight}

    @property
    def nodes(self):
        return list(self._adj.keys())

    @property
    def edges(self):
        return list(self._weights.keys())

    def add_node(self, node):
        if node not in self._adj:
            self._adj[node] = {}

    def add_edge(self, u, v, weight=1.0):
        """Add an undirected constraint edge between u and v."""
        if weight <= 0:
            raise ValueError(f"Edge weight must be positive, got {weight}")
        self.add_node(u)
        self.add_node(v)
        key = (u, v) if u <= v else (v, u)
        self._weights[key] = weight
        self._adj[u][v] = weight
        self._adj[v][u] = weight

    def weight(self, u, v):
        key = (u, v) if u <= v else (v, u)
        return self._weights.get(key, 0.0)

    def _dfs_spanning_tree(self, start):
        """Return (parent, tree_edges) from DFS."""
        parent = {start: None}
        tree_edges = set()
        stack = [start]
        while stack:
            cur = stack.pop()
            for nb in self._adj.get(cur, {}):
                if nb not in parent:
                    parent[nb] = cur
                    key = (cur, nb) if cur <= nb else (nb, cur)
                    tree_edges.add(key)
                    stack.append(nb)
        return parent, tree_edges

    def spanning_tree(self):
        """Return a set of tree edges (sorted tuples)."""
        if not self._adj:
            return set()
        _, tree_edges = self._dfs_spanning_tree(next(iter(self._adj)))
        return tree_edges

    def _find_path_in_tree(self, u, v, parent):
        """Find the unique path in the DFS tree between u and v.

        Returns list of nodes from u to v.
        """
        # Build ancestor chains for both nodes
        def chain(x):
            c = []
            while x is not None:
                c.append(x)
                x = parent[x]
            return c
        chain_u = chain(u)
        chain_v = chain(v)
        # Find LCA
        i_u = len(chain_u) - 1
        i_v = len(chain_v) - 1
        while i_u >= 0 and i_v >= 0 and chain_u[i_u] == chain_v[i_v]:
            i_u -= 1
            i_v -= 1
        # LCA is the node where the ancestor chains converge
        lca = chain_u[i_u + 1]
        # Path: u → ... → lca → ... → v
        # chain_u[0..i_u] = u down to just-below-LCA
        # chain_v[0..i_v] = v down to just-below-LCA
        path = chain_u[:i_u + 1] + [lca] + list(reversed(chain_v[:i_v + 1]))
        return path

    def fundamental_cycles(self):
        """Return list of fundamental cycles w.r.t. a spanning tree.

        Each cycle is a list of nodes forming a closed walk.
        """
        tree = self.spanning_tree()
        tree_set = set(tree)
        all_edges = set(self._weights.keys())

        # Build parent dict from tree
        if not self._adj:
            return []
        start = next(iter(self._adj))
        parent, _ = self._dfs_spanning_tree(start)
        # Handle disconnected components: do DFS from each unvisited node
        visited = set(parent.keys())
        for node in self._adj:
            if node not in visited:
                p2, _ = self._dfs_spanning_tree(node)
                parent.update(p2)
                visited.update(p2.keys())

        cycles = []
        for edge in all_edges:
            if edge in tree_set:
                continue
            u, v = edge
            # The unique path in tree + this edge forms a fundamental cycle
            path = self._find_path_in_tree(u, v, parent)
            if path:
                cycles.append(path)
        return cycles

    def holonomy(self, cycle):
        """Compute holonomy = product of weights around a cycle (as list of nodes)."""
        prod = 1.0
        n = len(cycle)
        for i in range(n):
            u = cycle[i]
            v = cycle[(i + 1) % n]
            prod *= self.weight(u, v)
        return prod

    def check_consensus(self, tolerance=0.01):
        """Check zero holonomy consensus.

        Returns (consensus: bool, violations: list of cycles).
        """
        violations = []
        for cycle in self.fundamental_cycles():
            h = self.holonomy(cycle)
            if abs(h - 1.0) > tolerance:
                violations.append((cycle, h))
        return len(violations) == 0, violations
