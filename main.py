import networkx as nx
import math
from typing import Callable, Dict, List, Tuple
import csv
import os
import pygame
import sys
import asyncio






# ---------------- FUNCION HEURISTICA ---------------- #
"""
La funcion heuristica: por distancia haversine
"""
def h_distanciaHaversiana(grafo:nx.Graph, nodoActual, nodoObjetivo):
   
    lat1 = grafo.nodes[nodoActual]['lat']
    lon1 = grafo.nodes[nodoActual]['lon']
    lat2 = grafo.nodes[nodoObjetivo]['lat']
    lon2 = grafo.nodes[nodoObjetivo]['lon']
    return haversine_distance(lat1, lon1, lat2, lon2)





# ---------------- ALGORITMO A* ---------------- #
'''
Algoritmo A*

@Params:
    * graph: grafo no dirigido con pesos. 
            Los vertices deben de tener un atributo llamado 'nombre', donde cada 'nombre' es unico
            Las aristas deben tener el atributo 'peso'
    * heuristic: la funcion heuristica
    * start_vertex: el atributo 'nombre' del vertice de inicio
    * end_vertex: el atributo 'nombre' del vertice meta

    reuturn una lista de todos nodos expanditos por orden, y lista de camino optimo (no limpiado en nuestro caso)
'''

def AStar(graph: nx.Graph, heuristic: Callable[[nx.Graph, str, str], float], start_vertex: str, end_vertex: str) -> Tuple[List[str], List[str]]:

    # Para simplificar cálculos, si el origen es igual al destino simplemente devolvemos la respuesta esperada...
    if start_vertex == end_vertex: 
        return [start_vertex], [start_vertex]

    # Esta es una lista que contiene todos los nodos (EN ORDEN) visitados por el algoritmo
    intermediatePath: List[str] = []

    # Primero obtenemos el vértice inicial y el vértice objetivo
    start = start_vertex
    goal = end_vertex

    # Conjunto de nodos descubiertos que pueden ser (re)explorados
    openSet: List[str] = [start]

    # Para un vértice 'n', cameFrom[n] es el nodo que lo precede en el camino de menor coste desde el inicio
    cameFrom: Dict[str, str] = dict()

    # El gScore de un nodo hace referencia al coste del camino más barato desde 'start' hasta ese nodo
    # Implementamos gScore como un diccionario {vértice : gScore}
    gScore: Dict[str, float] = {node: float('inf') for node in graph.nodes} 
    gScore[start] = 0

    # El fScore de un nodo indica "qué tan bueno" es el mejor camino hacia el final si pasa por ese nodo
    fScore: Dict[str, float] = {node: float('inf') for node in graph.nodes}
    fScore[start] = heuristic(graph, start, goal)

    # Mientras openSet no esté vacío...
    while len(openSet) != 0:

        # Nodo que vamos a expandir. Es el que tiene el menor valor de fScore dentro de openSet
        current = min(openSet, key=fScore.get)

        intermediatePath.append(current)

        # Si el nodo actual es el objetivo, ya encontramos la ruta óptima
        if current == goal: 
            # Ruta final, óptima     
            finalRoute = getPath(cameFrom, current, start)
            return intermediatePath, finalRoute

        # Quitamos el nodo que estamos expandiendo del openSet (equivalente a añadirlo a una "lista cerrada")
        openSet.remove(current)

        # Ahora expandimos el nodo:
        adjacent = graph.neighbors(current)  # Objeto iterable con los IDs de los vecinos del vértice actual

        for neighbor in adjacent:
            # Obtenemos el peso de la arista que conecta 'current' y 'neighbor'
            edge_data = graph[current][neighbor]
            weight = edge_data.get('weight', 1) 

            # Calculamos el nuevo gScore para cada vecino. Si es menor al que tenía antes,
            # significa que encontramos un camino mejor hacia ese nodo vecino.
            new_gScore = gScore[current] + weight
            
            if new_gScore < gScore[neighbor]:
                # Actualizamos la información del vecino con el nuevo camino más corto
                cameFrom[neighbor] = current
                gScore[neighbor] = new_gScore
                fScore[neighbor] = new_gScore + heuristic(graph, start, goal)

                # Si el vecino no estaba en openSet, lo añadimos (para seguir explorando desde él)
                if neighbor not in openSet:
                    openSet.append(neighbor)

        

    # Si no encontramos un camino, devolvemos una lista vacía para indicar que no existe ruta
    return [], [] 



# ---------------- FUNCION PARA OBTENER EL CAMINO OPTIMO LIMPIO---------------- #
'''
Funcion para comprimir los nodos (estaciones) duplicados del camino óptimo, y eliminar los nodos temporales
(los duplicados fue una estrategia para tener en cuenta el tiempo de transbordo miantras buscando el camino óptimo con A*)
(los nodos temporales fue creados si el origen o destino es un transbordo)
Ejemplo: [..., Zapata_12, Zapata_3, ...] --> [..., Zapata, Zapata, ...] --> [..., Zapata, ...]
         [..., Mixcoac_12, ...] --> [..., Mixcoac, ...]
'''
def caminoOptimo_limpio(camino):
    ruta_limpio = []
    num_paradas : int = 0 #Todas excepto la última estación(va a ser usado en la funcion tiempoCaminoOptimo)

    # Paso 1: quitar parte "_número" de cada nodo
    rutaAux = [nodo.split('_')[0] for nodo in camino]

    # Paso 2: comprimir nodos consecutivos repetidos (solo añadimos el nodo si es distinto al anterior) y ignorar nodos temporales
    for nodo in rutaAux:
        if nodo in ['ORIGEN', 'DESTINO']:
            continue  # ignorar nodos temporales
        if not ruta_limpio or ruta_limpio[-1] != nodo:
            ruta_limpio.append(nodo)

    num_paradas = max(0, len(ruta_limpio)-1)

    return ruta_limpio, num_paradas





# ---------------- FUNCIONES AUXILIARES ---------------- #

'''
Funcion para calcular la distanca heversiana:
Calcula la distancia del círculo máximo entre dos puntos de la Tierra (especificados en grados decimales)
'''
def haversine_distance(lat1, lon1, lat2, lon2):

    lon1, lat1, lon2, lat2 = map(math.radians, [lon1, lat1, lon2, lat2])

    dlon = lon2 - lon1 
    dlat = lat2 - lat1 
    a = math.sin(dlat/2)**2 + math.cos(lat1) * math.cos(lat2) * math.sin(dlon/2)**2
    c = 2 * math.asin(math.sqrt(a)) 
    r = 6371000 
    return c * r



'''
Function that reconstructs the path from 'current' to 'start' node and dictionary indicating 
the "parent" node of each node in the path.
The path is a list "path" that contains IN ORDER (from the starting vertex to the finish vertex) the names
of the vertices from the path
Reconstruye el camino desde el nodo actual hasta el inicio.
'''
def getPath(cameFrom: Dict[str, str], current: str, start: str) -> List[str]:
    path = [current]
    nextVertexInPath = current
    while nextVertexInPath != start:
        parent = cameFrom[nextVertexInPath]
        path.insert(0, parent)
        nextVertexInPath = parent
    return path



'''
Calcula longitud de un camino optimo en kilometro 
(usarlo con camino óptimo no limpiado todavía dado por A*)
'''
def lengthCaminoOptimo(grafo : nx.Graph, camino):
    distancia_total = 0
    # Recorre pares consecutivos de nodos para obtener el peso entre ellos
    for i in range(len(camino) - 1):
        nodo1 = camino[i]
        nodo2 = camino[i+1]
        #El peso entre dos nodos duplicados consecutivos no cuanta para calcular longitud del camino óptimo
        if(("_" in nodo1 and nodo1.split("_")[-1].isdigit()) and 
           ("_" in nodo2 and nodo2.split("_")[-1].isdigit())): #combrobar que son duplicados, si lo son, lo ignoramos
            continue
        else:
            distancia_total += grafo[nodo1][nodo2]["weight"] # metro(m)
    return round(distancia_total/1000, 2)




'''
Calcula tiempo de viaje (en minutos) de un camino optimo con una velocidad constante (km/h) 
(usarlo con camino óptimo dado por A* pero no limpiado todavia)
'''
def tiempoCaminoOptimo(grafo : nx.Graph, camino, vel:float, num_paradas:int):
    distancia_total = 0
    TIEMPO_PARADA_MIN = 0.4 #Tiempo de espera en cada parada en minuto (la parada origen incluido)

    # Recorre pares consecutivos de nodos para obtener el peso entre ellos
    for i in range(len(camino) - 1):
        distancia_total += grafo[camino[i]][camino[i+1]]["weight"] # metro(m)

    distancia_total = round(distancia_total/1000, 2)
    tiempo = (distancia_total/vel) * 60 # minutos
    return round(tiempo + TIEMPO_PARADA_MIN*num_paradas)
    




"""
Prepara el grafo para origen y destino que sean transbordos.
Conecta los duplicados de cada estación con aristas de peso 0 y crea nodos temporales ORIGEN_TEMP y DESTINO_TEMP para A*.

    Returns:
        origen_temp: nodo temporal de origen
        destino_temp: nodo temporal de destino
"""

def preparar_grafo_transbordos(grafo: nx.Graph, origen: str, destino: str, transbordos: list):
    
    # --- Nodo temporal de origen ---
    #Si el origen es un transbordo
    if origen in transbordos:
        nodos_origen = [n for n in grafo.nodes if n.startswith(origen)]
        origen_temp = 'ORIGEN_TEMP'
        #Crear nodo temporal
        lat = grafo.nodes[nodos_origen[0]]['lat']
        lon = grafo.nodes[nodos_origen[0]]['lon']
        grafo.add_node(origen_temp, lat=lat, lon=lon)
        #Conectar nodo temporal con los duplicados con peso 0
        for n in nodos_origen:
            grafo.add_edge(origen_temp, n, weight=0)
    else:
        origen_temp = origen

    # --- Nodo temporal de destino ---
    #Si el origen es un transbordo
    if destino in transbordos:
        nodos_destino = [n for n in grafo.nodes if n.startswith(destino)]
        destino_temp = 'DESTINO_TEMP'
        #Crear nodo temporal
        lat = grafo.nodes[nodos_destino[0]]['lat']
        lon = grafo.nodes[nodos_destino[0]]['lon']
        grafo.add_node(destino_temp, lat=lat, lon=lon)
        #Conectar nodo temporal con los duplicados con peso 0
        for n in nodos_destino:
            grafo.add_edge(destino_temp, n, weight=0)
    else:
        destino_temp = destino

    return origen_temp, destino_temp


"""
Elimina los nodos temporales creados en la preparación del grafo.
"""
def eliminar_nodos_temporales(grafo: nx.Graph):
    for nodo in ['ORIGEN_TEMP', 'DESTINO_TEMP']:
        if nodo in grafo:
            grafo.remove_node(nodo)



############### COMPROBACIONES ############################
'''
if __name__ == "__main__":

    transbordos = ["Mixcoac", "Zapata", "Centro Médico", "Balderas", "Tacubaya"]
    
    metro_grafo = metroCDMX.getMetro()

    origen = "Observatorio"
    destino = "Coyoacán"
    origen_temp, destino_temp = preparar_grafo_transbordos(metro_grafo, origen, destino, transbordos)
    nodosExpantidos, camino = AStar(metro_grafo, h_distanciaHaversiana, origen_temp, destino_temp)
    print("Nodos expantidos en orden:", nodosExpantidos, "\n")
    print("Nodos expantidos en orden:", camino, "\n")
    print("Camino optimo:", caminoOptimo_limpio(camino)[0], "\n")
    numParada = caminoOptimo_limpio(camino)[1]
    print("Longtud(km): ", lengthCaminoOptimo(metro_grafo, camino), "  Tiempo(min):", tiempoCaminoOptimo(metro_grafo, camino, 22, numParada), "\n")
    eliminar_nodos_temporales(metro_grafo)
'''



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


    # Crear nodos (todas las estaciones excepto los transbordos)
    transbordos = {"Mixcoac", "Zapata", "Centro Médico", "Balderas", "Tacubaya"}
    for fila in coords:
        nombre = fila['Nombre']
        if nombre not in transbordos:
            grafo.add_node(
                nombre,
                lat=float(fila['Latitud']),
                lon=float(fila['Longitud'])
            )
    
    #Crear nodos duplicados o triplicados para los transbordos 
    # (los nodos duplicados representa misma posicion fisica (mismas coordenadas), la heurística no se distorsiona)
    grafo.add_node("Mixcoac_7", lat=19.375891, lon=-99.187531)
    grafo.add_node("Mixcoac_12", lat=19.375891, lon=-99.187531)
    grafo.add_node("Zapata_3", lat=19.370952, lon=-99.164937)
    grafo.add_node("Zapata_12", lat=19.370952, lon=-99.164937)
    grafo.add_node("Centro Médico_3", lat=19.406637, lon=-99.155753)
    grafo.add_node("Centro Médico_9", lat=19.406637, lon=-99.155753)
    grafo.add_node("Balderas_1", lat=19.42744, lon=-99.149036)
    grafo.add_node("Balderas_3", lat=19.42744, lon=-99.149036)
    grafo.add_node("Tacubaya_1", lat=19.403439, lon=-99.187102) #Tacubaya es transbordo entre tres líneas, hay que triplicarlo
    grafo.add_node("Tacubaya_7", lat=19.403439, lon=-99.187102)
    grafo.add_node("Tacubaya_9", lat=19.403439, lon=-99.187102)


    # Crear aristas
    for fila in conexiones:
        grafo.add_edge(
            fila['Origen'],
            fila['Destino'],
            weight=float(fila['Peso'])
        )
    
    # Con velocidad 22km/h = 367m/min: 
    # Tiempo de cambio entre linea 7 y 12 en Mixcoac: conexión entre Mixcoac_7 y Mixcoac_12 --> 7min, equivale a 2569m (peso)
    # Conexión entre Zapata_3 y Zapata_12 --> 6min, equivale a 2202m 
    # Conexión entre Centro Médico_3 y Centro Médico_9 --> 4min, equivale a 1468m 
    # Conexión entre Balderas_1 y Balderas_3 --> 4min, equivale a 1468m 
    # Conexión entre Tacubaya_1 y Tacubaya_7 --> 6min, equivale a 2202m 
    # Conexión entre Tacubaya_1 y Tacubaya_9 --> 6min, equivale a 2202m 
    # Conexión entre Tacubaya_7 y Tacubaya_9 --> 6min, equivale a 2202m 


    return grafo   

async def initGUI(grafo, algorithm: Callable, heuristic : Callable):
#async def initGUI(grafo, algorithm: Callable[[nx.Graph, str, str], Tuple[List[str], List[str]]], heuristic : Callable[[nx.Graph, str, str], float]):
     
    #Lista de transbordos:
    transbordos = ["Mixcoac", "Zapata", "Centro Médico", "Balderas", "Tacubaya"]

    # Lista de todas estaciones originales de la mapa (sin nodos tuplicados)
    nombres = ['Observatorio', 'Tacubaya', 'Juanacatlán', 'Chapultepec', 'Sevilla', 'Insurgentes', 'Cuauhtémoc', 'Balderas', 
               'Universidad', 'Copilco', 'Miguel Ángel de Quevedo', 'Viveros', 'Coyoacán', 'Zapata', 'División del Norte', 
               'Eugenia', 'Etiopía', 'Centro Médico', 'Hospital General', 'Niños Héroes', 'Juárez', 'Barranca del Muerto', 
               'Mixcoac', 'San Antonio', 'San Pedro de los Pinos', 'Constituyentes', 'Auditorio', 'Polanco', 'Patriotismo', 
               'Chilpancingo', 'Lázaro Cárdenas', 'Insurgentes Sur', 'Hospital 20 de Noviembre', 'Parque de los Venados', 'Eje Central']


    
    # Lista con las coordenadas de las estaciones 
    # (orden coincidente con el orden de lista nombres)
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
        (722,574)]#Eje Central



    
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
    img_path = os.path.join(os.path.dirname(__file__), "assets")
    print(img_path)

    # Cargar la imagen del mapa
    metroMap = pygame.image.load(os.path.join(os.path.dirname(__file__), "assets", "metroMapCDMX.png")).convert()
    # Boton start
    startButton = pygame.image.load(os.path.join(img_path, "start.png")).convert_alpha()
    # Ajusta el tamano del boton a (100, 50) pixeles
    startButton = pygame.transform.scale(startButton, (100, 50))
    # Crea un rectángulo de colisión del mismo tamaño que la imagen de boton start, necesario para detectar clics
    startButtonRect = startButton.get_rect()
    # Posicion donde el boton va a estar colocado
    startButtonPos = (100,710)
    # Mueve el rectángulo a la misma posicion
    startButtonRect = startButtonRect.move(startButtonPos)

    # Boton reset (hacer lo mismo que boton start)
    resetButton = pygame.image.load(os.path.join(img_path, "reset.png")).convert_alpha()
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


    # ------------ VARIABLES PARA DATOS DE RUTA ------------
    # Las inicializamos vacías fuera del bucle principal
    origen_temp = ""
    destino_temp = ""
    str_info_dist = ""
    str_info_tiempo = ""
    num_parada : int = 0

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


                                # -------Aquí calculamos ruta optima con algoritmo A* y con la función caminoOptimo_limpio------------
                                # 1. Prepara el grafo para origen y destino que sean transbordos
                                origen_temp, destino_temp = preparar_grafo_transbordos(grafo, origin, destiny, transbordos)
                                # 2. Obtener ruta óptima limpia y final
                                intermediatePath, route = algorithm(grafo, heuristic, origen_temp, destino_temp) #It returns an intermediatePath (the steps taken by the algorithm) and a route optimo 
                                route_final, num_parada = caminoOptimo_limpio(route) #route_final es la ruta final, óptima y limpia

                                # 3. Calculo de tiempo y distancia (usando camino no limpiado)
                                str_info_dist = f"Distancia: {lengthCaminoOptimo(grafo, route)} km"
                                str_info_tiempo = f"Tiempo: {tiempoCaminoOptimo(grafo, route, 22, num_parada)} min"

                                # 4. Elimina los nodos temporales creados en la preparación del grafo, para dejarlo como antes
                                eliminar_nodos_temporales(grafo)
                            
                                # IMPRIMIR EN TERMINAL
                                print("\nResultados del cálculo:")
                                print(str_info_dist)
                                print(str_info_tiempo)
                                print("Camino optimo no limpiado:", route)
                                print("Número de paradas intermedias: ", num_parada-1)
                                print("-----------------------\n")


                                # Phase change
                                phase=3

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
            # In this loop se demuestra por orden las estaciones uno a uno
            print("Ruta optima: ", route_final)
            for station in route_final:
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
            for station in route_final:
                pygame.draw.circle(screen,(255, 255, 0),coords[nombres.index(station)],7)
                pygame.draw.circle(screen,(0,0,0),coords[nombres.index(station)],3)
            # We draw the reset button
            screen.blit(resetButton, resetButtonPos)


            # B) Dibujamos el Rectángulo Blanco
            caja_x = 180 
            caja_y = 660  
            ancho_caja = 300
            alto_caja = 70
            pygame.draw.rect(screen, (255, 255, 255), (caja_x, caja_y, ancho_caja, alto_caja))
            pygame.draw.rect(screen, (0, 0, 0), (caja_x, caja_y, ancho_caja, alto_caja), 2) # Borde

            # C) Renderizamos el texto
            if str_info_dist == "": str_info_dist = "Error: Sin datos"
            
            texto_distancia = font.render(str_info_dist, True, (0, 0, 0))
            texto_tiempo = font.render(str_info_tiempo, True, (0, 0, 0))

            # D) Pegamos el texto (usamos caja_y para que se mueva junto con la caja)
            screen.blit(texto_distancia, (caja_x + 10, caja_y + 10))
            screen.blit(texto_tiempo, (caja_x + 10, caja_y + 35))

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

async def main():
    # Obtenemos los datos creando un grafo del metro
    print("Cargando datos del metro...")
    metro_grafo = getMetro()


    # Lanzamos la interfaz grafica (VISTA)
    # Le pasamos el grafo (para pintar estaciones), la funcion Astar (para calcular rutas) y la funcion heuristica
    print("Iniciando interfaz grafica...")
    await initGUI(metro_grafo, AStar, h_distanciaHaversiana)

    # Bucle infinito final para mantener vivo el proceso async (requerido por pygbag)
    while True:
        await asyncio.sleep(0)

if __name__ == "__main__":
    asyncio.run(main())