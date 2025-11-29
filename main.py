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
    
    # Lista con los nombres de las estacioness
    nombres : List[str] = list(grafo.nodes) #Convierte NodeView a lista
    
    # Lista con las coordenadas de las estaciones
    coords:List[Tuple[int]] = [
        (161,382),#Observatorio
        (221,323),#Tacubaya
        (280,264),#Juanacatlan
        (324,220),#Chapultepec
        (368,175),#Sevilla
        (412,131),#Insurgentes
        (471,131),#Cuauhtemoc
        (530,131),#Balderas
        (530,751),#Universidad
        (530,707),#Copilco
        (530,663),#M. A. de Quevedo
        (530,618),#Viveros
        (530,574),#Coyoacan
        (530,515),#Zapata
        (530,471),#División del Norte
        (530,426),#Eugenia
        (530,383),#Etiopia
        (530,323),#Centro Medico
        (530,220),#Hospital General
        (530,176),#Niños Heroes
        (530,57),#Juarez
        (221,588),#Barranca del Muerto
        (221,515),#Mixcoac 
        (221,456),#San Antonio
        (221,412),#San Pedro de los Pinos
        (221,249),#Constituyentes
        (221,175),#Auditorio
        (221,101),#Polanco
        (339,323),#Patriotismo
        (442,323),#Chilpancingo
        (619,323),#Lazaro Cardenas
        (309,515),#Insuegentes Sur
        (412,515),#Hospital 20 de Noviembre
        (633,515),#Parque de los Venados
        (722,574)#Eje Central
        ]
    
    
   # ------ Inicialización de Pygame y carga de imágenes ------

    #Inicializa todos los módulos de Pygame que se necesitan
    pygame.init()

    # Tamano de la mapa del metro
    size = (959,800)

    #Crea la ventana donde se dibujará todo
    screen = pygame.display.set_mode((size))
    pygame.display.set_caption("Metro CDMX") # Titulo de la ventana que aparece arriba


    # Images:

    # Obtener ruta a la carpeta de imagenes
    base_path = os.path.dirname(__file__)
    assets_path = os.path.join(base_path, "assets")
    
    print("Buscando imágenes en:", assets_path)

    # Cargar la imagen del mapa
    metroMap = pygame.image.load(os.path.join(assets_path, "metroMapCDMX.png")).convert()
    # Boton start
    startButton = pygame.image.load(os.path.join(assets_path, "start.png")).convert_alpha() # Ajusta el tamano del boton a (100, 50) pixeles
    startButton = pygame.transform.scale(startButton, (100, 50))
    # Crea un rectángulo de colisión del mismo tamaño que la imagen de boton start, necesario para detectar clics
    startButtonRect = startButton.get_rect()
    # Posicion donde el boton va a estar colocado
    startButtonPos = (100,710)
    # Mueve el rectángulo a la misma posicion
    startButtonRect = startButtonRect.move(startButtonPos)

    # Boton reset (hacer lo mismo que boton start)
    resetButton = pygame.image.load(os.path.join(assets_path, "reset.png")).convert_alpha()
    resetButton = pygame.transform.scale(resetButton, (50, 50))
    resetButtonRect = resetButton.get_rect()
    resetButtonPos = (100,710)
    resetButtonRect = resetButtonRect.move(resetButtonPos)

 
    # Dibujar los elementos en la ventana
    screen.blit(metroMap, (0, 0)) #Dibuja la imagen del mapa en la esquina superior izquierda
    screen.blit(startButton, startButtonPos) #Dibuja el botón Start encima del mapa


    # Crea un objeto fuente para dibujar texto en la pantalla
    font = pygame.font.Font('freesansbold.ttf', 25)

    # We render the texts
    text1 = font.render('Seleccione un origen', True, (0,0,0), (255,255,255))
    text2 = font.render('Seleccione un destino', True, (0,0,0), (255,255,255))
    text3 = font.render('Mostrando ruta...', True, (0,0,0), (255,255,255))

    # We update the screen
    # (flip updates the whole surface)
    pygame.display.flip()

    # ------------ UI LOOP con fases -------------

    # Declaración de la variable fase
    phase=0

    # while True: bucle infinito, mantiene la ventana abierta hasta que el usuario cierre el programa
    # 5 fases:
    # Fase 0: boton start no ha sido clicked todavía (esperar a que el usuario presione Start)
    # Fase 1: botón start ya ha sido clicked, hay que seleccionar estación de origen
    # Fase 2: seleccionar estación de destino
    # Fase 3: mostrar pasos de A*
    # Fase 4: mostrar ruta final y botón Reset. Si clicked, ir a fase 1

    while True:

        # Fase 0: boton start no ha sido clicked todavía
        if phase==0:
            # Recorrer todos los eventos que ocurren (clics, teclado, cerrar ventana, etc)
            for event in pygame.event.get():

                # If theres a click:
                if event.type == pygame.MOUSEBUTTONUP: 
                    pos = pygame.mouse.get_pos()
                    # Verificar si el click fue en el buton start
                    if startButtonRect.collidepoint(pos): #verifica si el clic fue dentro del rectangulo de la imagen botón Start

                        # Update the screen
                        # (blit draws an image on top of another)
                        screen.blit(metroMap, (0, 0))
                        screen.blit(text1, (100,710))

                        # We draw the circles in the metro stations
                        for i in coords:
                            pygame.draw.circle(screen,(0,0,0),i,6)
                            pygame.draw.circle(screen,(255,255,255),i,3)
                            
                        pygame.display.flip()

                        # Cambia fase
                        phase=1

                # Si el usuario cerró la ventana
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()


        # Fase 1: botón start ya ha sido clicked, hay que seleccionar estación de origen
        elif phase==1:
                for event in pygame.event.get():
                    if event.type == pygame.MOUSEBUTTONUP:
                        pos = pygame.mouse.get_pos()

                        # We iterate through all the coords
                        for i in coords:
                            # If click distance to the ith-station is less than 5, we detect a collision
                            if abs(i[0]-pos[0])<=6 and abs(i[1]-pos[1])<=6:

                                # Save origin station
                                origin = nombres[coords.index(i)]
                                print("Origen seleccionado: ", origin)
                                # Origin coords
                                coordsOrigin = i

                                # Actuaizar la ventana
                                screen.blit(metroMap, (0, 0))
                                screen.blit(text2, (100,710))

                                # We redraw the station points, except for the origin
                                for o in coords:
                                    pygame.draw.circle(screen,(0,0,0),o,6)
                                    pygame.draw.circle(screen,(255,255,255),o,3)

                                # We redraw the origin in a different style
                                pygame.draw.circle(screen,(0,0,0),i,7)
                                pygame.display.flip()

                                # Phase change
                                phase =2
                                break

                    # Si el usuario cerró la ventana      
                    if event.type == pygame.QUIT:
                        pygame.quit()
                        sys.exit()

        # Fase 2: seleccionar estación de destino
        elif phase==2:
                for event in pygame.event.get():
                    if event.type == pygame.MOUSEBUTTONUP:
                        pos = pygame.mouse.get_pos()
                        for i in coords:
                            # Collision detection
                            if abs(i[0]-pos[0])<=6 and abs(i[1]-pos[1])<=6:

                                # Save destiny station
                                destiny = nombres[coords.index(i)]
                                print("Destiny seleccionado: ", destiny)
                                # Destiny coords
                                coordsDestiny = i
                                # Redraw todos los elementos necesarios y actualizar la ventana
                                screen.blit(metroMap, (0, 0))

                                # We redraw the station points, except for the destiny
                                for o in coords:
                                    pygame.draw.circle(screen,(0,0,0),o,6)
                                    pygame.draw.circle(screen,(255,255,255),o,3)

                                # We redraw the origin and the destiny in a different style
                                pygame.draw.circle(screen,(0,0,0),coordsOrigin,7)
                                pygame.draw.circle(screen,(0,0,0),coordsDestiny,7)

                                screen.blit(text3, (100,710))
                                pygame.display.flip()

                                # Phase change
                                phase=3

                                # Now we call the algorithm.
                                # It returns an intermediatePath (the steps taken by the algorithm)
                                # and a route (the final, optimal route)
                                ############ intermediatePath = 
                                route = algorithm(origin, destiny)

                                break

                    # Si el usuario cerró la ventana
                    if event.type == pygame.QUIT:
                        pygame.quit()
                        sys.exit()

        # Fase 3: mostrar pasos de A*
        elif phase==3:

            for event in pygame.event.get():
                 # Si el usuario cerró la ventana
                 if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()
            
            # Draw lights until all the stations are drawn.
            # In this loop we show the intermidiate steps of the algorithm
            ####### for station in intermediatePath:
            print("Ruta optima: ", route)
            for station in route:
                await asyncio.sleep(0.5) # Time between drawings of stations
                pygame.draw.circle(screen,(255, 51, 153),coords[nombres.index(station)],7)
                pygame.draw.circle(screen,(0,0,0),coords[nombres.index(station)],3)
                pygame.display.flip()
                
            
            # phase change
            phase=4
        
        # Fase 4: mostrar ruta final y botón Reset. Si clicked, ir a fase 1
        elif phase==4:
            # We reset the drawings
            screen.blit(metroMap, (0, 0))
            
            # We draw the (final) path
            for station in route:
                pygame.draw.circle(screen,(255, 255, 0),coords[nombres.index(station)],7)
                pygame.draw.circle(screen,(0,0,0),coords[nombres.index(station)],3)
            # We draw the reset button
            screen.blit(resetButton, resetButtonPos)

            # We update the screen
            pygame.display.flip()

            # We are now going to check if the reset button has been clicked
            for event in pygame.event.get():
                # If theres a click:
                if event.type == pygame.MOUSEBUTTONUP:
                    pos = pygame.mouse.get_pos()
                    # We check if the click was in the reset button
                    if resetButtonRect.collidepoint(pos): # This checks if the click was inside the rectangle of the image
                        # We reset the GUI
                        screen.blit(metroMap, (0, 0))
                        screen.blit(startButton, startButtonPos)
                        pygame.display.flip()
                        phase=0 #Cambia fase a cero para empezar el proceso de nuevo
                
                # Si el usuario cerró la ventana
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()

        await asyncio.sleep(0)

    """ # Variables de estado
    phase = 0
    origin = ""
    destiny = ""
    coordsOrigin = (0,0)
    coordsDestiny = (0,0)
    route = []

    # UI LOOP
    while True:
        if phase==0:
            for event in pygame.event.get():
                if event.type == pygame.MOUSEBUTTONUP:
                    pos = pygame.mouse.get_pos()
                    if startButtonRect.collidepoint(pos):
                        screen.blit(metroMap, (0, 0))
                        screen.blit(text1, ButtonPos)
                        for i in coords:
                            pygame.draw.circle(screen,(0,0,0),i,6)
                            pygame.draw.circle(screen,(255,255,255),i,3)
                        pygame.display.flip()
                        phase=1
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()

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
                            screen.blit(text2, (250,30))
                            for o in coords:
                                pygame.draw.circle(screen,(0,0,0),o,6)
                                pygame.draw.circle(screen,(255,255,255),o,3)
                            pygame.draw.circle(screen,(0,0,0),i,7)
                            pygame.display.flip()
                            phase = 2
                            break
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()

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
                            screen.blit(text3, (300,60))
                            pygame.display.flip()
                            
                            phase=3
                            # LLAMADA AL ALGORITMO
                            route = algorithm(origin, destiny)
                            break
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()

        elif phase==3:
            for event in pygame.event.get():
                 if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()
            
            # Animacion de la ruta
            for station in route:
                await asyncio.sleep(0.2) 
                idx = nombres.index(station)
                pygame.draw.circle(screen,(255, 51, 153),coords[idx],7)
                pygame.draw.circle(screen,(0,0,0),coords[idx],3)
                pygame.display.flip()
            
            phase=4
        
        elif phase==4:
            screen.blit(metroMap, (0, 0))
            for station in route:
                idx = nombres.index(station)
                pygame.draw.circle(screen,(255, 255, 0),coords[idx],7)
                pygame.draw.circle(screen,(0,0,0),coords[idx],3)
            screen.blit(resetButton, ButtonPos)
            pygame.display.flip()

            for event in pygame.event.get():
                if event.type == pygame.MOUSEBUTTONUP:
                    pos = pygame.mouse.get_pos()
                    if resetButtonRect.collidepoint(pos):
                        screen.blit(metroMap, (0, 0))
                        screen.blit(startButton, ButtonPos)
                        pygame.display.flip()
                        phase=0
                if event.type == pygame.QUIT:
                    pygame.quit()
                    sys.exit()

        await asyncio.sleep(0) """








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