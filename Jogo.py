# Representa o tabuleiro do 8-puzzle
class Puzzle:
    def __init__(self, state):
        self.state = state
        self.size = int(len(state) ** 0.5)  # Determina o tamanho do tabuleiro (3x3, por exemplo)
        self.zero_position = self.state.index(0)  # Encontra a posição do espaço vazio

    def is_goal(self, goal_state):
        # Verifica se o estado atual é o objetivo
        return self.state == goal_state

    def get_neighbors(self):
        # Retorna uma lista de vizinhos possíveis (novos estados após mover o espaço vazio)
        neighbors = []
        zero_row = self.zero_position // self.size
        zero_col = self.zero_position % self.size
        
        # Definir possíveis movimentos para o espaço vazio
        moves = [
            ("up", zero_row > 0),  # Mover para cima
            ("down", zero_row < self.size - 1),  # Mover para baixo
            ("left", zero_col > 0),  # Mover para a esquerda
            ("right", zero_col < self.size - 1)  # Mover para a direita
        ]
        
        for move, condition in moves:
            if condition:
                new_zero_position = self.zero_position
                if move == "up":
                    new_zero_position -= self.size
                elif move == "down":
                    new_zero_position += self.size
                elif move == "left":
                    new_zero_position -= 1
                elif move == "right":
                    new_zero_position += 1
                
                # Troca o espaço vazio com a posição desejada
                new_state = list(self.state)
                new_state[self.zero_position], new_state[new_zero_position] = new_state[new_zero_position], new_state[self.zero_position]
                
                # Adiciona o novo estado à lista de vizinhos
                neighbors.append(Puzzle(new_state))
        
        return neighbors

# Busca em Largura (BFS)
def bfs(initial_puzzle, goal_state):
    queue = [initial_puzzle]
    visited = set()
    parent_map = {tuple(initial_puzzle.state): None}  # Map para rastrear pais de cada estado
    
    while queue:
        current_puzzle = queue.pop(0)  # Remove o primeiro elemento da lista
        
        if current_puzzle.is_goal(goal_state):
            # Caminho para a solução encontrado, constrói a sequência de movimentos
            return reconstruct_path(parent_map, current_puzzle, initial_puzzle)
        
        # Adiciona estado atual aos visitados
        visited.add(tuple(current_puzzle.state))
        
        for neighbor in current_puzzle.get_neighbors():
            if tuple(neighbor.state) not in visited:
                # Adiciona o vizinho à fila
                queue.append(neighbor)
                
                # Mapeia o vizinho para o estado atual como seu pai
                parent_map[tuple(neighbor.state)] = current_puzzle
    
    return None

# Busca em Profundidade (DFS)
def dfs(initial_puzzle, goal_state):
    stack = [initial_puzzle]
    visited = set()
    parent_map = {tuple(initial_puzzle.state): None}  # Map para rastrear pais de cada estado
    
    while stack:
        current_puzzle = stack.pop()  # Remove o último elemento da lista
        
        if current_puzzle.is_goal(goal_state):
            # Caminho para a solução encontrado, constrói a sequência de movimentos
            return reconstruct_path(parent_map, current_puzzle, initial_puzzle)
        
        # Adiciona estado atual aos visitados
        visited.add(tuple(current_puzzle.state))
        
        for neighbor in current_puzzle.get_neighbors():
            if tuple(neighbor.state) not in visited:
                # Adiciona o vizinho à pilha
                stack.append(neighbor)
                
                # Mapeia o vizinho para o estado atual como seu pai
                parent_map[tuple(neighbor.state)] = current_puzzle
    
    return None

# Função para reconstruir o caminho a partir de um mapa de pais
def reconstruct_path(parent_map, goal_puzzle, initial_puzzle):
    path = []
    current_puzzle = goal_puzzle
    
    while current_puzzle != initial_puzzle:
        path.append(current_puzzle)
        current_puzzle = parent_map[tuple(current_puzzle.state)]
    
    # Inverte o caminho para mostrar na ordem correta
    path.reverse()
    return path

# Implementação das heurísticas

# Distância de Manhattan
def manhattan_distance(puzzle, goal_state):
    total_distance = 0
    size = puzzle.size
    
    for i, value in enumerate(puzzle.state):
        if value != 0:
            goal_index = goal_state.index(value)
            current_row, current_col = i // size, i % size
            goal_row, goal_col = goal_index // size, goal_index % size
            total_distance += abs(current_row - goal_row) + abs(current_col - goal_col)
    
    return total_distance

# Distância de Hamming
def hamming_distance(puzzle, goal_state):
    return sum(1 for i in range(len(puzzle.state)) if puzzle.state[i] != goal_state[i] and puzzle.state[i] != 0)

# Busca Best First (Greedy) com base em heurísticas
def greedy_best_first(initial_puzzle, goal_state, heuristic):
    # Usamos uma lista para armazenar estados ordenados por heurística
    frontier = [(0, initial_puzzle)]
    visited = set()
    parent_map = {tuple(initial_puzzle.state): None}
    
    while frontier:
        # Ordena a lista de fronteira com base no custo heurístico
        frontier.sort(key=lambda x: x[0])
        current_puzzle = frontier.pop(0)[1]  # Remove o estado com menor custo
        
        if current_puzzle.is_goal(goal_state):
            # Caminho para a solução encontrado, constrói a sequência de movimentos
            return reconstruct_path(parent_map, current_puzzle, initial_puzzle)
        
        # Adiciona estado atual aos visitados
        visited.add(tuple(current_puzzle.state))
        
        for neighbor in current_puzzle.get_neighbors():
            if tuple(neighbor.state) not em visited:
                heuristic_cost = heuristic(neighbor, goal_state)
                
                # Adiciona o vizinho à lista de fronteira
                frontier.append((heuristic_cost, neighbor))
                
                # Mapeia o vizinho para o estado atual como seu pai
                parent_map[tuple(neighbor.state)] = current_puzzle
    
    return None

# Exemplo de uso

def main():
    # Estado inicial (você pode ler do usuário)
    initial_state = [1, 2, 3, 4, 5, 6, 7, 8, 0]  # Exemplo de estado inicial (vazio na última posição)
    initial_puzzle = Puzzle(initial_state)

    # Estado final desejado
    goal_state = [1, 2, 3, 4, 5, 6, 7, 8, 0]  # Exemplo de estado final (vazio na última posição)
    
    # Exemplo de busca em largura
    path_bfs = bfs(initial_puzzle, goal_state)
    if path_bfs:
        print("Solução usando BFS:")
        for puzzle in path_bfs:
            print(puzzle.state)
    
    # Exemplo de busca em profundidade
    path_dfs = dfs(initial_puzzle, goal_state)
    if path_dfs:
        print("Solução usando DFS:")
        for puzzle in path_dfs:
            print(puzzle.state)
    
    # Exemplo de busca gulosa com heurística de Manhattan
    path_greedy_manhattan = greedy_best_first(initial_puzzle, goal_state, manhattan_distance)
    if path_greedy_manhattan:
        print("Solução usando Greedy com Manhattan Distance:")
        for puzzle in path_greedy_manhattan:
            print(puzzle.state)

    # Exemplo de busca gulosa com heurística de Hamming
    path_greedy_hamming = greedy_best_first(initial_puzzle, goal_state, hamming_distance)
    if path_greedy_hamming:
        print("Solução usando Greedy com Hamming Distance:")
        for puzzle in path_greedy_hamming:
            print(puzzle.state)

if __name__ == "__main__":
    main()
