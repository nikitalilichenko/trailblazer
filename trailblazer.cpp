/* Nikita Lilichenko
 * HW 7 - Trailblazer
 * Section Leader - Eli Echt-Wilson
 * This project uses 5 different types of algorithms to create trails from one point to another in different worlds
 * (maps, mazes, terrains), with varying algorithms dealing with factors such as, path efficiency, and path
 * heuristics. There is also a maze-building aspect that takes in Kruskal's Algorithm to form a minimum spanning
 * tree.
 */

#include "trailblazer.h"
#include "basicgraph.h"
#include "set.h"
#include "queue.h"
#include "map.h"
#include "pqueue.h"

using namespace std;

//Function prototypes
Vector<Vertex*> createPath(Map<Vertex*, Vertex*> previousVerteces, Vertex* start, Vertex* end);
bool edgeEndpointsConnected(Set<Set<Vertex*>>& connectedVerteceszx, Vertex* endVertex, Vertex* currentVertex);
void markConnected(Set<Set<Vertex*>>& connectedVerteces, Vertex* a, Vertex* b);

/* function serves as recursive function for Depth First Search algorithm. It passes in the graph, the Vector path,
 * and the Set visitedVerteces by reference, as well as the currentVertex and the end vertex.
 */
bool depthFirstSearchHelper(BasicGraph& graph, Vector<Vertex*>& path, Set<Vertex*>& visitedVerteces, Vertex* currentVertex, Vertex* end) {
    /* the function begins by adding the currentVertex to the final path and visitedVerteces, and checks to see
     * if that vertex is the end vertex. If it is, the function returns true.
     */
    path.add(currentVertex);
    currentVertex->setColor(GREEN);
    if (currentVertex == end) {
        return true;
    }
    visitedVerteces.add(currentVertex);
    /* for each neighbor of the current vertex, the function makes sure that it has not been visited yet, then
     * proceeds to call itself to find a path. If a path is found, true is returned, if not the currentNeighbor
     * color is set to gray.
     */
    for (Vertex* currentNeighbor : graph.getNeighbors(currentVertex)) {
        if (visitedVerteces.contains(currentNeighbor)) {
            continue;
        }
        bool searchSuccessful = depthFirstSearchHelper(graph, path, visitedVerteces, currentNeighbor, end);
        if (searchSuccessful) {
            return true;
        }
        else {
            currentNeighbor->setColor(GRAY);
        }
    }
    /* if no path is found, the path removes the back vertex and returns false.
     */
    path.removeBack();
    return false;
}

/* Function implements the Depth First Search algorithm, which aims to successfully find any path between the
 * start and end verteces. It first resets the graph's data, then calls its recursive helper after defining path
 * and visitedVerteces. It then returns the path.
 */
Vector<Vertex*> depthFirstSearch(BasicGraph& graph, Vertex* start, Vertex* end) {
    graph.resetData();
    Vector<Vertex*> path;
    Set<Vertex*> visitedVerteces;
    depthFirstSearchHelper(graph, path, visitedVerteces, start, end);
    return path;
}

/* Function impements the Breadth First Search, which searches each neighbor of the current vertex individually
 * instead of finding a path immediately. The function begins by resetting data, then defining necessary structures
 * for the functions execution, as well as adding the start vertex to visitedVerteces and vertecesToVisit.
 */
Vector<Vertex*> breadthFirstSearch(BasicGraph& graph, Vertex* start, Vertex* end) {
    graph.resetData();
    Queue<Vertex*> vertexesToVisit;
    Set<Vertex*> visitedVerteces;
    Map<Vertex*, Vertex*> previousVerteces;
    vertexesToVisit.enqueue(start);
    visitedVerteces.add(start);
    start->setColor(GREEN);
    Vertex* currentVertex = start;
    /* This while loop checks if the end vertex has been visited and if vertexesToVisit is empty. If neither is the
     * case, then the function dequeues the next in queue and adds it to visitedVerteces, while also changing color.
     */
    while (!visitedVerteces.contains(end) && !vertexesToVisit.isEmpty()) {
        currentVertex = vertexesToVisit.dequeue();
        visitedVerteces.add(currentVertex);
        currentVertex->setColor(GREEN);
        /* This for loop cylces through all verteces of graph, and if it had not been visited, then it is enqueued
         * and added to the previousVerteces map to later create a path.
         */
        for (Vertex* currentNeighbor : graph.getVertexSet()) {
            if (!visitedVerteces.contains(currentNeighbor)) {
                currentNeighbor->setColor(YELLOW);
                vertexesToVisit.enqueue(currentNeighbor);
                previousVerteces.put(currentNeighbor, currentVertex);
            }
        }
    }
    /* the function createPath is returned to create the final path
     */
    return createPath(previousVerteces, start, end);
}

/* Function implements Dijkstra's Algorithm, which is similar to a Breadth First Search Algorithm but also utilizes
 * the cost of taking a certain path. The algorithm also uses a Priority queue instead of a regular queue to queue
 * up verteces. The first section of the function defines the necessary daat structures to run the program, then
 * enqueues the start vertex, and adds it to the VertexCosts map.
 */
Vector<Vertex*> dijkstrasAlgorithm(BasicGraph& graph, Vertex* start, Vertex* end) {
    graph.resetData();
    PriorityQueue<Vertex*> vertexesToVisit;
    Set<Vertex*> visitedVerteces;
    Map<Vertex*, double> vertexCosts;
    Map<Vertex*, Vertex*> previousVerteces;
    vertexesToVisit.enqueue(start, 0);
    vertexCosts[start] = 0;
    /* This while loop checks if the end vertex has been visited and if vertexesToVisit is empty. If neither is the
     * case, then the function dequeues the next in queue and adds it to visitedVerteces, while also changing color.
     */
    while (!visitedVerteces.contains(end) && !vertexesToVisit.isEmpty()) {
        Vertex* currentVertex = vertexesToVisit.dequeue();
        visitedVerteces.add(currentVertex);
        currentVertex->setColor(GREEN);
        /* This for loop cylces through all verteces of graph, and if it had not been visited, then it is enqueued
         * and added to the previousVerteces map to later create a path. However, unlike BFS, this loop also checks
         * if the cost of the edge between the current vertex and the current neighbor has a cos tless of that then
         * the current cost of the that vertex. Only then does it is add this vertex to the previous verteces map.
         */
        for (Vertex* currentNeighbor : graph.getNeighbors(currentVertex)) {
            if (!visitedVerteces.contains(currentNeighbor)) {
                double cost = vertexCosts.get(currentVertex) + graph.getEdge(currentVertex, currentNeighbor)->cost;
                if (!vertexCosts.containsKey(currentNeighbor) || cost < vertexCosts.get(currentNeighbor)) {
                    currentNeighbor->setColor(YELLOW);
                    vertexesToVisit.enqueue(currentNeighbor, cost);
                    vertexCosts[currentNeighbor] = cost;
                    previousVerteces.put(currentNeighbor, currentVertex);
                }
            }
        }
    }
    /* the function createPath is returned to create the final path
     */
    return createPath(previousVerteces, start, end);
}

//TODO figure out path cost bug(use print statements (dont want to enqueue previous heuristics with currentNeighbor))

/* Function serves as helper function for the aStar function, and passes in all parameters as the main aStar function
 * as well as an edge that monitors which edge should be excluded from the search (for alternate path). The function
 * implements the A* algorithm, which utilizes both a priority queue and heuristic function. The function begins by
 * creating all necessary data structures, and enqueuing the start vertex to the queue, while listing its cost as 0
 * in the map.
 */
Vector<Vertex*> aStar(BasicGraph& graph, Vertex* start, Vertex* end, Edge* toBeExcluded) {
    PriorityQueue<Vertex*> vertexesToVisit;
    Set<Vertex*> visitedVerteces;
    Map<Vertex*, double> vertexCosts;
    Map<Vertex*, Vertex*> previousVerteces;
    Map<Vector<Vertex*>, double> pathCosts;
    vertexesToVisit.enqueue(start, heuristicFunction(start, end));
    vertexCosts[start] = 0;
    /* While loop runs while the end vertex has not been visited yet and the queue is not empty, and begins by dequeuing a
     * vertex, which it marks as visited.
     */
    while (!visitedVerteces.contains(end) && !vertexesToVisit.isEmpty()) {
        Vertex* currentVertex = vertexesToVisit.dequeue();
        visitedVerteces.add(currentVertex);
        currentVertex->setColor(GREEN);
        /* For every neighbor of the recently dequeued vertex, the function checks if the toBeExcluded edge is not a null
         * pointer and if it is the edge between the current neighbor and current vertex. If it meets both of these conditions,
         * the loop continues.
         */
        for (Vertex* currentNeighbor : graph.getNeighbors(currentVertex)) {
            if (toBeExcluded != nullptr
                    && ((toBeExcluded->start == currentVertex
                    && toBeExcluded->end == currentNeighbor)
                    || (toBeExcluded->start == currentVertex
                    && toBeExcluded->end == currentNeighbor))) {
                continue;
            }
            /* if the current neighbor has not been visited, the funciton calculates the heuristic cost between the current
             * neighbor and the end, as well as the cost of the current vertex plus teh edge between the current neighbor and
             * the current vertex plus the heuristic cost.
             */
            if (!visitedVerteces.contains(currentNeighbor)) {
                double heuristicCost = heuristicFunction(currentNeighbor, end);
                double cost = (vertexCosts.get(currentVertex) + graph.getEdge(currentVertex, currentNeighbor)->cost);
                /* If the current neighbor is not in the vertex costs map and the current cost is less than the cost of the
                 * current neighbor, the neighbor is enqueued with this new cost, and added to the map, as well as placed into
                 * the previousVerteces map to be later made into a path.
                 */
                if (!vertexCosts.containsKey(currentNeighbor) || cost < vertexCosts.get(currentNeighbor)) {
                    currentNeighbor->setColor(YELLOW);
                    vertexesToVisit.enqueue(currentNeighbor, cost + heuristicCost);
                    vertexCosts[currentNeighbor] = cost;
                    previousVerteces.put(currentNeighbor, currentVertex);
                }
            }
        }
    }
    /* the function createPath is returned to create the final path
     */
    return createPath(previousVerteces, start, end);
}

/* Function implements the A* Algorithm. It resets the graph's data, then returns its aStar helper function.
 */
Vector<Vertex*> aStar(BasicGraph& graph, Vertex* start, Vertex* end) {
    graph.resetData();
    return aStar(graph, start, end, nullptr);
}

/* Function implements the alternate path algorithm, which uses A* but ignores a certain edge at a time to be able to form
 * a next-best alternate path. It implements all of the same parameters as aStar but also inlcudes a double the is used to
 * establish if an alternate path is significantly different from the best A* path. The function begins by establishing a
 * vector of vectors, then calls aStar to form a best path to iterate over.
 */
Vector<Vertex*> alternatePath(BasicGraph& graph, Vertex* start, Vertex* end, double difference) {
    graph.resetData();
    Vector<Vector<Vertex*> > alternatePaths;
    Vector<Vertex*> bestPath = aStar(graph, start, end);
    if (bestPath.size() == 0) {
        return bestPath;
    }
    /* This for loop itierates over every single vertex in the best path created by aStar. For every vertex, an edge is initialzed
     * between the current vertex and the previous vertex, and this edge is passed through into the aStar helper function to
     * ignore it. The resulting path is added to the vector of vectors, which holds all of the alternate paths.
     */
    Vertex* previousVertex = nullptr;
    for (Vertex* currentVertex : bestPath) {
        if (previousVertex == nullptr) {
            previousVertex = currentVertex;
            continue;
        }
        Edge* currentEdge = graph.getEdge(previousVertex, currentVertex);
        Vector<Vertex*> currentAlternatePath = aStar(graph, start, end, currentEdge);
        alternatePaths.add(currentAlternatePath);
        previousVertex = currentVertex;
    }
    /* This for loop itierates over all of the alternate paths in the large vector, and for every vector the function calculates
     * the differences of vertex betweeen the current alternate path and the best path. The best alternate path that is significantly
     * different from the best path is returned as the final path.
     */
    Vector<Vertex*> currentBestAlternatePath;
    for (Vector<Vertex*> currentAlternatePath : alternatePaths) {
        Set<Vertex*> exclusiveVerteces;
        for (int i = 0; i < currentAlternatePath.size(); i++) {
            if (!bestPath.contains(currentAlternatePath[i])) {
                exclusiveVerteces.add(currentAlternatePath[i]);
            }
        }
        double alternateDifference = ((double)exclusiveVerteces.size())/bestPath.size();
        if (alternateDifference > difference) {
            currentBestAlternatePath = currentAlternatePath;
        }
    }
    return currentBestAlternatePath;
}

/* Function is used in a variety of functions to create the path (Vector of verteces) after all of the substantial code is
 * run. The function takes in the previousVerteces map, the start vertex, and the end vertex.
 */
Vector<Vertex*> createPath(Map<Vertex*, Vertex*> previousVerteces, Vertex* start, Vertex* end){
    /* Function sets the current vertex to end and defines the final path to be returned
     */
    Vertex* currentVertex = end;
    Vector<Vertex*> path;
    /* In this while loop, the function inserts a vertex to the front of the path starting from the end. If the vertex being
     * currently examined is the start vertex, the loop breaks after adding the start to the path. CurrentVertex is then set to
     * the next vertex in the map. If the end vertex is a null pointer, an empty vector is returned.
     */
    while (true) {
        path.insert(0, currentVertex);
        if (currentVertex == start) {
            break;
        }
        currentVertex = previousVerteces.get(currentVertex);
        if (currentVertex == nullptr) {
            Vector<Vertex*> noPath;
            return noPath;
        }
    }
    return path;
}

/* Function implements Kruskal's Algorithm, which is used to create a random maze by creating a minimum spannning tree. It
 * passes in a BasicGraph and utilizes a Priority Queue. "Large" Maze runtime: ~10 seconds
 */
Set<Edge*> kruskal(BasicGraph& graph) {
    /* Several necessary data structures are called, then a for loop is run to enqueue all edges based on their cost
     */
    PriorityQueue<Edge*> edges;
    Set<Set<Vertex*>> connectedVerteces;
    Set<Edge*> finalMazeEdges;
    for (Edge* currentEdge : graph.getEdgeSet()) {
        edges.enqueue(currentEdge, currentEdge->cost);
    }
    /* While loop is run while the queue is not empty, and function dequeues an edge, then calls a function to check if the
     * the two verteces between the edge are connected. If they are, the loop skips over the edge and it's not enqueued. If
     * they're not connected, the edge is added to the final set of edges and a function is called to mark them as connected.
     */
    while (!edges.isEmpty()) {
        Edge* currentEdge = edges.dequeue();
        bool endpointsConnected = edgeEndpointsConnected(connectedVerteces, currentEdge->finish, currentEdge->start);
        if (endpointsConnected) {
            continue;
        }
        finalMazeEdges.add(currentEdge);
        markConnected(connectedVerteces, currentEdge->start, currentEdge->finish);
    }
    return finalMazeEdges;
}

/* Function is used to determine if two verteces are connected. A set of sets of verteces is passed in along with the start
 * and end verteces. A loop for each set in the set is run, and if the current set contains both the start and end verteces
 * the function returns true. If the entire loop is run and true is not returned, the function returns false.
 */
bool edgeEndpointsConnected(Set<Set<Vertex*>>& connectedVerteces, Vertex* endVertex, Vertex* startVertex) {
    for (Set<Vertex*> currentSet : connectedVerteces) {
        if (currentSet.contains(startVertex) && currentSet.contains(endVertex)) {
            return true;
        }
    }
    return false;
}

/* Function is used to mark two verteces as being connected. The function passes in a set of sets of verteces, as well as two
 * verteces. The function begins by setting a vertex as a null pointer and creating a set.
 */
void markConnected(Set<Set<Vertex*>>& connectedVerteces, Vertex* a, Vertex* b) {
    Vertex* vertexToSearch = nullptr;
    Set<Vertex*> found;
    /* For loop is run for every set in the large set; if the vertexToSearch is not a null pointer and the current set contains
     * the vertex, the found set (which contains the earlier set with the vertex) merges with the current set, and the large
     * map removes the current set. This ends the function.
     */
    for (Set<Vertex*> currentSet : connectedVerteces) {
        if (vertexToSearch != nullptr) {
            if (currentSet.contains(vertexToSearch)) {
                connectedVerteces.remove(found);
                found.addAll(currentSet);
                connectedVerteces.add(found);
                connectedVerteces.remove(currentSet);
                return;
            }
        }
        /* If the earlier condition is not met, the function checks if the current set contains eitehr the a or b vertex. If either
         * is met, the function sets the other vertex as vertexToSearch. The found set is set equal to the current set.
         */
        else {
            if (currentSet.contains(a)) {
                vertexToSearch = b;
                found = currentSet;
            }
            else if(currentSet.contains(b)){
                vertexToSearch = a;
                found = currentSet;
            }
        }

    }
    /* If the loop is run through but there is never an instance where the vertexToSearch is found in the sets, the vertex is
     * added to found, and that is added to the main set.
     */
    if (vertexToSearch != nullptr) {
        connectedVerteces.remove(found);
        found.add(vertexToSearch);
        connectedVerteces.add(found);
    }
    /* If the vertexToVisit is never found, a new set containing a and b verteces is created and added to the main set.
     */
    else {
        Set<Vertex*> newVertexSet;
        newVertexSet.add(a);
        newVertexSet.add(b);
        connectedVerteces.add(newVertexSet);
    }
}

