import heapq
import matplotlib.pyplot as plt
import networkx as nx

def dijkstra(graph, start):
    # Inicialización
    distances = {node: float('infinity') for node in graph}
    distances[start] = 0
    priority_queue = [(0, start)]
    paths = {start: [start]}
    
    while priority_queue:
        current_distance, current_node = heapq.heappop(priority_queue)
        
        if current_distance > distances[current_node]:
            continue
            
        for neighbor, weight in graph[current_node].items():
            distance = current_distance + weight
            
            if distance < distances[neighbor]:
                distances[neighbor] = distance
                heapq.heappush(priority_queue, (distance, neighbor))
                paths[neighbor] = paths[current_node] + [neighbor]
    
    return distances, paths

def visualize_graph(graph, shortest_path=None):
    G = nx.Graph()
    
    for node in graph:
        for neighbor, weight in graph[node].items():
            G.add_edge(node, neighbor, weight=weight)
    
    pos = nx.spring_layout(G)
    nx.draw_networkx_nodes(G, pos, node_size=700)
    nx.draw_networkx_edges(G, pos, width=1.5)
    
    if shortest_path:
        path_edges = list(zip(shortest_path, shortest_path[1:]))
        nx.draw_networkx_edges(G, pos, edgelist=path_edges, width=3, edge_color='r')
    
    edge_labels = nx.get_edge_attributes(G, 'weight')
    nx.draw_networkx_edge_labels(G, pos, edge_labels=edge_labels)
    nx.draw_networkx_labels(G, pos, font_size=12, font_family='sans-serif')
    
    plt.axis('off')
    plt.show()

# Ejemplo de grafo
graph = {
    'A': {'B': 1, 'C': 4},
    'B': {'A': 1, 'C': 2, 'D': 5},
    'C': {'A': 4, 'B': 2, 'D': 1},
    'D': {'B': 5, 'C': 1}
}

# Ejecutar algoritmo
start_node = 'A'
distances, paths = dijkstra(graph, start_node)

# Mostrar resultados
print(f"Distancias desde {start_node}:")
for node, distance in distances.items():
    print(f"{node}: {distance} (Camino: {' -> '.join(paths[node])})")

# Visualizar grafo con el camino más corto a un nodo destino
dest_node = 'D'
visualize_graph(graph, paths[dest_node])
