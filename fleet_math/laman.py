"""Laman rigidity for 2D bar-joint frameworks.

Laman's theorem states that a generic bar-joint framework in 2D is
minimally rigid iff it has exactly E = 2V - 3 edges and every subset
of k vertices spans at most 2k - 3 edges (the Laman count condition).

The sufficient condition E >= 2V - 3 per connected component is the
first-order check used here.
"""

from .h1 import connected_components


def _count_subgraph_laman(graph, nodes_subset):
    """Count edges whose both endpoints lie in nodes_subset."""
    edge_set = nodes_subset
    count = 0
    if hasattr(graph, 'edges'):
        for u, v in graph.edges:
            if u in edge_set and v in edge_set:
                count += 1
    elif hasattr(graph, '_weights'):
        for (u, v) in graph._weights:
            if u in edge_set and v in edge_set:
                count += 1
    else:
        # Fallback: dict adjacency
        for u in edge_set:
            if hasattr(graph, 'get'):
                nbs = graph.get(u, {})
                for v in nbs:
                    if v in edge_set and u <= v:
                        count += 1
    return count


def _laman_count_holds(graph):
    """Check Laman's count condition: every k-vertex subset has ≤ 2k-3 edges.

    Only checks subsets up to |V| for small graphs; returns True for
    large graphs where exhaustive check is intractable.
    """
    if hasattr(graph, 'nodes'):
        all_nodes = list(graph.nodes)
    else:
        all_nodes = list(graph.keys())

    n = len(all_nodes)
    if n > 12:  # Skip exhaustive check for large graphs
        return True

    # Iterate over all non-trivial subsets (k >= 2)
    from itertools import combinations
    for k in range(2, n + 1):
        for subset in combinations(all_nodes, k):
            e = _count_subgraph_laman(graph, set(subset))
            if e > 2 * k - 3:
                return False
    return True


def _partition_by_component(graph):
    """Split nodes into connected components, return list of node sets."""
    if hasattr(graph, 'nodes'):
        all_nodes = list(graph.nodes)
    else:
        all_nodes = list(graph.keys())

    adj = {}
    for n in all_nodes:
        adj[n] = set()

    if hasattr(graph, '_adj'):
        # ConstraintGraph
        for u, nbs in graph._adj.items():
            adj[u] = set(nbs.keys())
    elif hasattr(graph, 'edges'):
        for u, v in graph.edges:
            adj[u].add(v)
            adj[v].add(u)
    elif hasattr(graph, 'items'):
        for u, nbs in graph.items():
            for v in nbs:
                adj[u].add(v)

    visited = set()
    components = []
    for n in all_nodes:
        if n in visited:
            continue
        comp = set()
        stack = [n]
        while stack:
            cur = stack.pop()
            if cur in visited:
                continue
            visited.add(cur)
            comp.add(cur)
            for nb in adj.get(cur, set()):
                if nb not in visited:
                    stack.append(nb)
        components.append(comp)
    return components


def count_vertices(graph):
    if hasattr(graph, 'nodes'):
        return len(list(graph.nodes))
    return len(list(graph.keys()))


def count_edges(graph):
    if hasattr(graph, 'edges'):
        return len(list(graph.edges))
    if hasattr(graph, '_weights'):
        return len(graph._weights)
    return 0


def is_rigid(graph):
    """Check if a graph satisfies the Laman rigidity necessary condition.

    A connected graph is rigid if E >= 2V - 3.
    For disconnected graphs, each component must satisfy the condition.
    """
    comps = _partition_by_component(graph)
    for comp in comps:
        k = len(comp)
        if k <= 2:
            # 0, 1, or 2 nodes: always rigid (trivially)
            continue
        e = _count_subgraph_laman(graph, comp)
        if e < 2 * k - 3:
            return False
    return True


def is_minimally_rigid(graph):
    """Check if graph is minimally rigid.

    For a connected graph: E == 2V - 3 (first-order condition).
    Also checks the Laman count condition for small graphs.
    """
    if connected_components(graph) != 1:
        return False

    V = count_vertices(graph)
    E = count_edges(graph)

    if V <= 2:
        return True

    if E != 2 * V - 3:
        return False

    # For small graphs, verify the Laman count condition
    if V <= 12:
        return _laman_count_holds(graph)

    return True


def rigid_margin(graph):
    """Compute margin relative to the rigidity threshold.

    Returns E - (2V - 3) for a connected graph.
    Negative means not rigid, zero means minimally rigid,
    positive means redundantly rigid.
    """
    if connected_components(graph) != 1:
        # For disconnected graphs, return sum of margins per component
        comps = _partition_by_component(graph)
        total_margin = 0
        for comp in comps:
            k = len(comp)
            if k <= 2:
                continue
            e = _count_subgraph_laman(graph, comp)
            total_margin += e - (2 * k - 3)
        return total_margin

    V = count_vertices(graph)
    E = count_edges(graph)
    if V <= 2:
        return 0
    return E - (2 * V - 3)
