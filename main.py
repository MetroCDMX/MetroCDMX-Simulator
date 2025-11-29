import networkx as nx
import math
from typing import Callable, Dict, List, Tuple
import csv
import os
import pygame
import sys
import asyncio



'''
MODULO A* CON NETWORKX
Corregido para coincidir con atributos 'lat' y 'lon' de metroCDMX.py
'''

# Variable global para almacenar el grafo
GRAFO_GLOBAL = None

# ---------------- FUNCIONES AUXILIARES ---------------- #

def haversine_distance(lat1: float, lon1: float, lat2: float, lon2: float) -> float:
    """Calcula la distancia Haversine (geodésica) en Kilómetros."""
    R = 6371.0  # Radio de la Tierra en km

    dlat = math.radians(lat2 - lat1)
    dlon = math.radians(lon2 - lon1)
    
    a = (math.sin(dlat / 2) ** 2 +
         math.cos(math.radians(lat1)) * math.cos(math.radians(lat2)) *
         math.sin(dlon / 2) ** 2)
    
    c = 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a))
    
    return R * c

def getPath(cameFrom: Dict[str, str], current: str, start: str) -> List[str]:
    """Reconstruye el camino desde el nodo actual hasta el inicio."""
    path = [current]
    nextVertexInPath = current
    while nextVertexInPath != start:
        parent = cameFrom[nextVertexInPath]
        path.insert(0, parent)
        nextVertexInPath = parent
    return path

# ---------------- HEURÍSTICAS ---------------- #

def MSTHeuristic(graph: nx.Graph, start: str, goal: str, current: str, openSet: List[str], cameFrom: Dict[str, str]):
    currentPath = getPath(cameFrom, current, start)
    edges_to_remove = [(currentPath[i-1], currentPath[i]) for i in range(1, len(currentPath))]

    g = graph.copy() 
    g.remove_edges_from(edges_to_remove)
    
    if g.is_directed():
        g = g.to_undirected()

    mst_edges = nx.minimum_spanning_edges(g, weight='weight', data=True)
    return sum(edge_data['weight'] for u, v, edge_data in mst_edges)

def djikstraHeuristic(graph: nx.Graph, start: str, goal: str, current: str, openSet: List[str], cameFrom: Dict[str, str]):
    currentPath = getPath(cameFrom, current, start)
    edges_to_remove = [(currentPath[i-1], currentPath[i]) for i in range(1, len(currentPath))]

    g = graph.copy()
    g.remove_edges_from(edges_to_remove)

    try:
        return nx.shortest_path_length(g, source=current, target=goal, weight='weight')
    except nx.NetworkXNoPath:
        return float('inf')

def euclideanDistanceHeuristic(graph: nx.Graph, start: str, goal: str, current: str, openSet: List[str], cameFrom: Dict[str, str]):
    """
    Implementa la distancia geodésica.
    Usa 'lat' y 'lon' para coincidir con tu metroCDMX.py
    """
    curr_data = graph.nodes[current]
    goal_data = graph.nodes[goal]
    
    # AQUÍ ESTABA EL ERROR: Cambiado de 'long' a 'lon'
    return haversine_distance(curr_data["lat"], curr_data["lon"], 
                              goal_data["lat"], goal_data["lon"])

# ---------------- ALGORITMO PRINCIPAL ---------------- #

def AStar(graph: nx.Graph, 
          heuristic: Callable, 
          start_vertex: str, 
          end_vertex: str) -> Tuple[List[str], List[str]]:

    if start_vertex == end_vertex: 
        return [start_vertex], [start_vertex]

    intermediatePath: List[str] = []
    start = start_vertex
    goal = end_vertex

    openSet: List[str] = [start]
    cameFrom: Dict[str, str] = dict()

    gScore: Dict[str, float] = {node: float('inf') for node in graph.nodes} 
    gScore[start] = 0

    fScore: Dict[str, float] = {node: float('inf') for node in graph.nodes}
    
    try:
        h_val = heuristic(graph, start, goal, start, openSet, cameFrom)
    except Exception:
        h_val = 0
    fScore[start] = h_val

    while len(openSet) != 0:
        current = min(openSet, key=fScore.get)
        intermediatePath.append(current)

        if current == goal:      
            finalRoute = getPath(cameFrom, current, start)
            return intermediatePath, finalRoute

        openSet.remove(current)

        for neighbor in graph.neighbors(current):
            edge_data = graph[current][neighbor]
            weight = edge_data.get('weight', 1) 

            new_gScore = gScore[current] + weight
            
            if new_gScore < gScore[neighbor]:
                cameFrom[neighbor] = current
                gScore[neighbor] = new_gScore
                fScore[neighbor] = new_gScore + heuristic(graph, start, goal, neighbor, openSet, cameFrom)

                if neighbor not in openSet:
                    openSet.append(neighbor)
    
    return [], []

# ---------------- ADAPTADORES UI ---------------- #

def inicializar_algoritmo(graph: nx.Graph):
    global GRAFO_GLOBAL
    GRAFO_GLOBAL = graph
    print(f"Grafo inicializado correctamente (Nodos: {len(graph.nodes)})")

def caminoOptimo(start_vertex: str, end_vertex: str) -> List[str]:
    if GRAFO_GLOBAL is None:
        print("Error: Grafo no inicializado.")
        return []

    # Llamamos al algoritmo con la heurística correcta
    intermediate, final = AStar(GRAFO_GLOBAL, euclideanDistanceHeuristic, start_vertex, end_vertex)
    return final




def getMetro():
    """
    Funcion para obtener grafo del metro, en el que los pesos son distancias reales entre dos estaciones
    """
    # Crear un grafo vacio
    grafo = nx.Graph()

    # Leer coordsCDMX.csv
    # Usamos os.path.dirname(__file__) para asegurar que busca en la misma carpeta que este script
    coordsPath = os.path.join(os.path.dirname(__file__), "coordsCDMX.csv")
    
    with open(coordsPath, newline="", encoding="utf-8") as file:
        reader = csv.DictReader(file)
        coords = list(reader)

    # Leer conexiones.csv
    conexionesPath = os.path.join(os.path.dirname(__file__), "conexiones.csv")
    
    with open(conexionesPath, newline="", encoding="utf-8") as file:
        reader = csv.DictReader(file)
        conexiones = list(reader)

    # Crear nodos
    for fila in coords:
        grafo.add_node(
            fila['Nombre'],
            lat=float(fila['Latitud']),
            lon=float(fila['Longitud'])
        )

    # Crear aristas
    for fila in conexiones:
        grafo.add_edge(
            fila['Origen'],
            fila['Destino'],
            weight=float(fila['Peso'])
        )

    return grafo




   


async def initGUI(grafo, algorithm: Callable[[str, str], List[str]]):
    
    # Lista con los nombres de las estaciones
    nombres : List[str] = list(grafo.nodes)
    
    # Lista con las coordenadas de las estaciones
    coords:List[Tuple[int]] = [
        (161,382), (221,323), (280,264), (324,220), (368,175), (412,131),
        (471,131), (530,131), (530,751), (530,707), (530,663), (530,618),
        (530,574), (530,515), (530,471), (530,426), (530,383), (530,323),
        (530,220), (530,176), (530,57), (221,588), (221,515), (221,456),
        (221,412), (221,249), (221,175), (221,101), (339,323), (442,323),
        (619,323), (309,515), (412,515), (633,515), (722,574)
    ]
    
    # ------ Inicialización de Pygame ------
    pygame.init()
    size = (959,800)
    screen = pygame.display.set_mode((size))
    pygame.display.set_caption("Metro CDMX")

    # ------ CONFIGURACIÓN DE RUTA DE IMÁGENES ------
    # Usamos os.path.abspath para obtener la ruta absoluta del archivo actual
    base_path = os.path.dirname(os.path.abspath(__file__))
    
    # Definimos explícitamente la carpeta "assets"
    assets_path = os.path.join(base_path, "assets")
    
    print(f"Buscando imágenes en: {assets_path}")

    # Función auxiliar para cargar imágenes de forma segura
    def cargar_imagen(nombre):
        ruta_completa = os.path.join(assets_path, nombre)
        if not os.path.exists(ruta_completa):
            print(f"❌ ERROR CRÍTICO: No encuentro el archivo {nombre} en {assets_path}")
            # Crear un cuadro rojo de error si falta la imagen para que no se cierre el programa
            surf = pygame.Surface((50, 50))
            surf.fill((255, 0, 0))
            return surf
        return pygame.image.load(ruta_completa)

    # Carga de imágenes usando la ruta definida
    metroMap = cargar_imagen("metroMapCDMX.png").convert()
    
    startButton = cargar_imagen("start.png").convert_alpha()
    startButton = pygame.transform.scale(startButton, (100, 50))
    startButtonRect = startButton.get_rect()
    startButtonPos = (100,710)
    startButtonRect = startButtonRect.move(startButtonPos)

    resetButton = cargar_imagen("reset.png").convert_alpha()
    resetButton = pygame.transform.scale(resetButton, (50, 50))
    resetButtonRect = resetButton.get_rect()
    resetButtonPos = (100,710)
    resetButtonRect = resetButtonRect.move(resetButtonPos)

    # -----------------------------------------------------------

    screen.blit(metroMap, (0, 0))
    screen.blit(startButton, startButtonPos)

    font = pygame.font.Font('freesansbold.ttf', 25)
    text1 = font.render('Seleccione un origen', True, (0,0,0), (255,255,255))
    text2 = font.render('Seleccione un destino', True, (0,0,0), (255,255,255))
    text3 = font.render('Mostrando ruta...', True, (0,0,0), (255,255,255))

    pygame.display.flip()

    # ------------ UI LOOP -------------
    phase=0
    
    # Variables globales para el texto
    str_info_dist = ""
    str_info_tiempo = ""

    while True:
        # Fase 0: Esperando Start
        if phase==0:
            for event in pygame.event.get():
                if event.type == pygame.MOUSEBUTTONUP: 
                    pos = pygame.mouse.get_pos()
                    if startButtonRect.collidepoint(pos):
                        screen.blit(metroMap, (0, 0))
                        screen.blit(text1, (100,710))
                        for i in coords:
                            pygame.draw.circle(screen,(0,0,0),i,6)
                            pygame.draw.circle(screen,(255,255,255),i,3)
                        pygame.display.flip()
                        phase=1

                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()

        # Fase 1: Seleccionar Origen
        elif phase==1:
                for event in pygame.event.get():
                    if event.type == pygame.MOUSEBUTTONUP:
                        pos = pygame.mouse.get_pos()
                        for i in coords:
                            if abs(i[0]-pos[0])<=6 and abs(i[1]-pos[1])<=6:
                                origin = nombres[coords.index(i)]
                                print("Origen seleccionado: ", origin)
                                coordsOrigin = i
                                screen.blit(metroMap, (0, 0))
                                screen.blit(text2, (100,710))
                                for o in coords:
                                    pygame.draw.circle(screen,(0,0,0),o,6)
                                    pygame.draw.circle(screen,(255,255,255),o,3)
                                pygame.draw.circle(screen,(0,0,0),i,7)
                                pygame.display.flip()
                                phase =2
                                break     
                    if event.type == pygame.QUIT:
                        pygame.quit()
                        sys.exit()

        # Fase 2: Seleccionar Destino y CÁLCULOS
        elif phase==2:
                for event in pygame.event.get():
                    if event.type == pygame.MOUSEBUTTONUP:
                        pos = pygame.mouse.get_pos()
                        for i in coords:
                            if abs(i[0]-pos[0])<=6 and abs(i[1]-pos[1])<=6:
                                destiny = nombres[coords.index(i)]
                                print("Destiny seleccionado: ", destiny)
                                coordsDestiny = i
                                
                                screen.blit(metroMap, (0, 0))
                                for o in coords:
                                    pygame.draw.circle(screen,(0,0,0),o,6)
                                    pygame.draw.circle(screen,(255,255,255),o,3)
                                pygame.draw.circle(screen,(0,0,0),coordsOrigin,7)
                                pygame.draw.circle(screen,(0,0,0),coordsDestiny,7)
                                screen.blit(text3, (100,710))
                                pygame.display.flip()
                                phase=3

                                route = algorithm(origin, destiny)

                                # --- CÁLCULOS ---
                                VELOCIDAD_MS = 11.11
                                TIEMPO_PARADA_MIN = 0.5
                                total_metros = 0
                                try:
                                    for k in range(len(route) - 1):
                                        total_metros += grafo[route[k]][route[k+1]]['weight']
                                except KeyError:
                                    total_metros = 0

                                tiempo_viaje_min = (total_metros / VELOCIDAD_MS) / 60
                                num_paradas = max(0, len(route) - 2)
                                tiempo_total_min = round(tiempo_viaje_min + (num_paradas * TIEMPO_PARADA_MIN), 1)
                                dist_km = round(total_metros / 1000, 2)

                                str_info_dist = f"Distancia: {dist_km} km"
                                str_info_tiempo = f"Tiempo aprox: {tiempo_total_min} min"
                                print(f"Ruta: {route}\n{str_info_dist}\n{str_info_tiempo}")
                                # ----------------
                                break

                    if event.type == pygame.QUIT:
                        pygame.quit()
                        sys.exit()

        # Fase 3: Animación
        elif phase==3:
            for event in pygame.event.get():
                 if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()
            
            for station in route:
                await asyncio.sleep(0.5)
                pygame.draw.circle(screen,(255, 51, 153),coords[nombres.index(station)],7)
                pygame.draw.circle(screen,(0,0,0),coords[nombres.index(station)],3)
                pygame.display.flip()
            
            phase=4
        
        # Fase 4: Resultado y Caja Blanca
        elif phase==4:
            screen.blit(metroMap, (0, 0))
            for station in route:
                pygame.draw.circle(screen,(255, 255, 0),coords[nombres.index(station)],7)
                pygame.draw.circle(screen,(0,0,0),coords[nombres.index(station)],3)
            screen.blit(resetButton, resetButtonPos)

            # --- DIBUJO CAJA BLANCA ---
            caja_x, caja_y = 170, 670
            ancho_caja, alto_caja = 300, 75

            pygame.draw.rect(screen, (255, 255, 255), (caja_x, caja_y, ancho_caja, alto_caja))
            pygame.draw.rect(screen, (0, 0, 0), (caja_x, caja_y, ancho_caja, alto_caja), 2)

            if str_info_dist == "": str_info_dist = "Calculando..."
            
            t_dist = font.render(str_info_dist, True, (0, 0, 0))
            t_time = font.render(str_info_tiempo, True, (0, 0, 0))

            screen.blit(t_dist, (caja_x + 10, caja_y + 10))
            screen.blit(t_time, (caja_x + 10, caja_y + 35))
            # --------------------------

            pygame.display.flip()

            for event in pygame.event.get():
                if event.type == pygame.MOUSEBUTTONUP:
                    pos = pygame.mouse.get_pos()
                    if resetButtonRect.collidepoint(pos):
                        screen.blit(metroMap, (0, 0))
                        screen.blit(startButton, startButtonPos)
                        pygame.display.flip()
                        str_info_dist = ""
                        str_info_tiempo = ""
                        phase=0
                
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()

        await asyncio.sleep(0)

    


async def main():
    # 1. Obtenemos los datos del metro
    print("Cargando datos del metro...")
    metro_grafo = getMetro()  # Ahora getMetro() está en el mismo archivo

    # 2. Inicializamos el algoritmo
    inicializar_algoritmo(metro_grafo)  # Ahora está en el mismo archivo

    # 3. Lanzamos la interfaz gráfica
    print("Iniciando interfaz gráfica...")
    await initGUI(metro_grafo, caminoOptimo)  # Ahora está en el mismo archivo

    # Bucle infinito
    while True:
        await asyncio.sleep(0)

if __name__ == "__main__":
    asyncio.run(main())