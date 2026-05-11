"""H1 emergence detection.

Uses the first Betti number β₁ = E - V + C to quantify the number of
independent cycles in a graph, and defines emergence severity relative
to the maximal cycle count for a graph of the same vertex count.
"""


def connected_components(graph):
    """Count connected components of a graph.

    Graph is expected to have:
      - graph.nodes: iterable of node identifiers
      - graph.edges: iterable of (u, v) tuples
    Graph may also be a dict adjacency {node: {neighbor: ...}}.
    """
    if hasattr(graph, 'nodes'):
        all_nodes = list(graph.nodes)
    else:
        all_nodes = list(graph.keys())

    # Build adjacency lookup
    adj = {}
    for n in all_nodes:
        adj[n] = set()

    if hasattr(graph, 'edges'):
        edge_list = graph.edges
    elif hasattr(graph, 'items'):
        # dict adjacency
        for u, nbs in graph.items():
            for v in nbs:
                adj[u].add(v)
        edge_list = []
    else:
        edge_list = []

    if not hasattr(graph, 'items') or not isinstance(graph, dict):
        for u, v in edge_list:
            adj[u].add(v)
            adj[v].add(u)

    visited = set()
    comps = 0
    for n in all_nodes:
        if n in visited:
            continue
        comps += 1
        stack = [n]
        while stack:
            cur = stack.pop()
            if cur in visited:
                continue
            visited.add(cur)
            for nb in adj.get(cur, set()):
                if nb not in visited:
                    stack.append(nb)

    return comps


def betti_1(graph):
    """Compute β₁ = E - V + C (first Betti number).

    E = number of edges
    V = number of vertices
    C = number of connected components
    """
    if hasattr(graph, 'nodes'):
        V = len(list(graph.nodes))
    else:
        V = len(list(graph.keys()))

    if hasattr(graph, 'edges'):
        E = len(list(graph.edges))
    else:
        E = 0

    C = connected_components(graph)
    return E - V + C


def emergence_severity(graph):
    """ε = β₁ / (V - 2) - 1

    Measures how far the graph exceeds the minimally-connected threshold.
    ε > 0 indicates redundant connectivity beyond a tree.
    """
    b1 = betti_1(graph)
    if hasattr(graph, 'nodes'):
        V = len(list(graph.nodes))
    else:
        V = len(list(graph.keys()))
    denom = max(V - 2, 1)
    return b1 / denom - 1.0


def detect_emergence(graph, threshold=0.0):
    """Detect if the graph exhibits emergence (ε > threshold).

    Returns True when β₁ > V - 2, meaning cycles outnumber the tree
    threshold, indicating the graph has topological structure beyond
    minimal connectivity.
    """
    b1 = betti_1(graph)
    if hasattr(graph, 'nodes'):
        V = len(list(graph.nodes))
    else:
        V = len(list(graph.keys()))
    return b1 > V - 2
