#include <iostream>
#include <vector>
#include <random>   // For random number generation
#include <chrono>   // For seeding the random number generator
#include <array>   // Para std::array
#include <utility> // Para std::pair

// Define Map as a vector of vectors of integers.
// You can change 'int' to whatever type best represents your cells (e.g., char, bool).
using Map = std::vector<std::vector<int>>;

/**
 * @brief Prints the map (matrix) to the console.
 * @param map The map to print.
 */
void printMap(const Map& map) {
    std::cout << "--- Current Map ---" << std::endl;
    for (const auto& row : map) {
        for (int cell : row) {
            // Adapt this to represent your cells meaningfully (e.g., ' ' for empty, '#' for occupied).
            std::cout << cell << " ";
        }
        std::cout << std::endl;
    }
    std::cout << "-------------------" << std::endl;
}

/**
 * @brief Function to implement the Cellular Automata logic.
 * It should take a map and return the updated map after one iteration.
 * @param currentMap The map in its current state.
 * @param W Width of the map.
 * @param H Height of the map.
 * @param R Radius of the neighbor window (e.g., 1 for 3x3, 2 for 5x5).
 * @param U Threshold to decide if the current cell becomes 1 or 0.
 * @return The map after applying the cellular automata rules.
 */
Map cellularAutomata(Map inputMap, int W, int H, int R, double U, int I) {
    Map prev = inputMap;   // copia de myMap
    Map next = inputMap;   // buffer para la siguiente generación

    for (int iter = 0; iter < I; ++iter) {
        // Calcula next a partir de prev
        for (int x = 0; x < H; ++x) {
            for (int y = 0; y < W; ++y) {
                int count = 0;
                for (int dx = -R; dx <= R; ++dx) {
                    for (int dy = -R; dy <= R; ++dy) {
                        if (dx == 0 && dy == 0) continue;
                        int nx = x + dx, ny = y + dy;
                        if (nx >= 0 && nx < H && ny >= 0 && ny < W)
                            count += prev[nx][ny];
                        else
                            count += 1;
                    }
                }
                int totalN = (2*R+1)*(2*R+1) - 1;
                next[x][y] = (count > U * totalN) ? 1 : 0;
            }
        }

        // Imprime esta sub-iteración
        std::cout << "--- CA Sub-iteration " << (iter+1) << " ---\n";
        printMap(next);

        // Swap rápido (O(1) en metadatos)
        prev.swap(next);
    }

    return prev;  // última generación
}

/**
 * @brief Function to implement the Drunk Agent logic.
 * It should take a map and parameters controlling the agent's behavior,
 * then return the updated map after the agent performs its actions.
 *
 * @param currentMap The map in its current state.
 * @param W Width of the map.
 * @param H Height of the map.
 * @param J The number of times the agent "walks" (initiates a path).
 * @param I The number of steps the agent takes per "walk".
 * @param roomSizeX Max width of rooms the agent can generate.
 * @param roomSizeY Max height of rooms the agent can generate.
 * @param probGenerateRoom Probability (0.0 to 1.0) of generating a room at each step.
 * @param probIncreaseRoom If no room is generated, this value increases probGenerateRoom.
 * @param probChangeDirection Probability (0.0 to 1.0) of changing direction at each step.
 * @param probIncreaseChange If direction is not changed, this value increases probChangeDirection.
 * @param agentX Current X position of the agent (updated by reference).
 * @param agentY Current Y position of the agent (updated by reference).
 * @return The map after the agent's movements and actions.
 */
Map drunkAgent(const Map& currentMap, int W, int H, int J, int I, int roomSizeX, int roomSizeY,
               double probGenerateRoom, double probIncreaseRoom,
               double probChangeDirection, double probIncreaseChange,
               int& agentX, int& agentY) {
    Map newMap = currentMap; // The new map is a copy of the current one

std::mt19937 rng(std::chrono::steady_clock::now().time_since_epoch().count());
    std::uniform_real_distribution<double> probDist(0.0, 1.0);
    std::uniform_int_distribution<int> dirDist(0, 3); // 0: up, 1: right, 2: down, 3: left

    // Direcciones de movimiento: {dx, dy}
    std::array<std::pair<int, int>, 4> directions = {
        std::make_pair(-1, 0), // arriba
        std::make_pair(0, 1),  // derecha
        std::make_pair(1, 0),  // abajo
        std::make_pair(0, -1)  // izquierda
    };

    double roomProb = probGenerateRoom;
    double changeDirProb = probChangeDirection;

    for (int j = 0; j < J; ++j) {
        // Elige dirección inicial aleatoria
        int dir = dirDist(rng);

        for (int i = 0; i < I; ++i) {
            // Marca el paso del agente (pasillo)
            if (agentX >= 0 && agentX < H && agentY >= 0 && agentY < W) {
                newMap[agentX][agentY] = 1;
            }

            // ¿Genera habitación?
            if (probDist(rng) < roomProb) {
                int startX = std::max(0, agentX - roomSizeY / 2);
                int endX = std::min(H, agentX + (roomSizeY + 1) / 2);
                int startY = std::max(0, agentY - roomSizeX / 2);
                int endY = std::min(W, agentY + (roomSizeX + 1) / 2);

                for (int x = startX; x < endX; ++x) {
                    for (int y = startY; y < endY; ++y) {
                        newMap[x][y] = 1;
                    }
                }

                roomProb = probGenerateRoom; // Reset
            } else {
                roomProb += probIncreaseRoom;
            }

            // ¿Cambia dirección?
            if (probDist(rng) < changeDirProb) {
                dir = dirDist(rng);
                changeDirProb = probChangeDirection; // Reset
            } else {
                changeDirProb += probIncreaseChange;
            }

            // Movimiento del agente
            int newX = agentX + directions[dir].first;
            int newY = agentY + directions[dir].second;

            // Verifica límites
            if (newX >= 0 && newX < H && newY >= 0 && newY < W) {
                agentX = newX;
                agentY = newY;
            } else {
                // Si se sale del mapa, cambia de dirección
                dir = dirDist(rng);
            }
        }
    }
    return newMap;
}

int main() {
    std::cout << "--- CELLULAR AUTOMATA AND DRUNK AGENT SIMULATION ---\n";

    // --- Initial Map Configuration ---
    int mapRows = 10, mapCols = 20;
    Map myMap(mapRows, std::vector<int>(mapCols, 0));

    // (Aquí inicializas myMap con el patrón que quieras)
    // ...

    std::cout << "\nInitial map state:\n";
    printMap(myMap);

    // --- Simulation Parameters ---
    int numIterations = 5;

    // Parámetros CA
    int ca_W = mapCols, ca_H = mapRows, ca_R = 1, ca_I = 3;
    double ca_U = 0.5;

    // Parámetros Drunk Agent (ejemplo)
    int da_W = mapCols, da_H = mapRows, da_J = 5, da_I = 10;
    int da_roomSizeX = 5, da_roomSizeY = 3;
    double da_probGenerateRoom = 0.1, da_probIncreaseRoom = 0.05;
    double da_probChangeDirection = 0.2, da_probIncreaseChange = 0.03;
    int drunkAgentX = mapRows/2, drunkAgentY = mapCols/2;

    // --- Main Simulation Loop ---
    for (int iteration = 0; iteration < numIterations; ++iteration) {
        std::cout << "\n=== Iteration " << iteration+1 << " ===\n";

        // 1) Generamos un nuevo mapa CA SIN TOCAR myMap
        Map caMap = cellularAutomata(myMap, ca_W, ca_H, ca_R, ca_U, ca_I);

        // 2) Luego aplicamos el agente al mapa resultante de CA
        Map daMap = drunkAgent(caMap, da_W, da_H, da_J, da_I,
                              da_roomSizeX, da_roomSizeY,
                              da_probGenerateRoom, da_probIncreaseRoom,
                              da_probChangeDirection, da_probIncreaseChange,
                              drunkAgentX, drunkAgentY);

        // 3) Imprimimos el resultado de esta iteración
        printMap(daMap);

        // si quisieras que la siguiente iteración parta de daMap,
        // podrías hacer: myMap = daMap;
        // pero myMap original sigue intacto si no lo reasignas.
    }

    std::cout << "\n--- Simulation Finished ---\n";
    return 0;
}