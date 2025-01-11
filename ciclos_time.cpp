#include <bits/stdc++.h>
#include <chrono>  // Biblioteca para medir o tempo

using namespace std;
using namespace chrono;  // Adicione essa linha para simplificar o código com `chrono`

#define FOR(i, a) for (int i = 0; i < (int)a; i++)
#define PB push_back
#define all(x) x.begin(), x.end()
#define rall(x) x.rbegin(), x.rend()
#define F first
#define S second

typedef long long ll;
typedef pair<int, int> pii;
typedef vector<int> vi;

const int INF = 0x3f3f3f3f;
const int MAX = 1e5 + 7;
const int SQ = sqrt(MAX);

unordered_map<int, string> vertexIndices;
vector<pair<int, int>> graph[int(1e6)];
vector<vector<int>> adjList(int(1e6));  // Lista de adjacência para detecção de ciclos

void addVertice(int indice, string nome) {
  vertexIndices[indice] = nome;
}

void addAresta(int u, int v, int wt) {
  graph[u].PB(make_pair(v, wt));
  adjList[u].PB(v);  // Adiciona na lista de adjacência para detecção de ciclos
}

void Dijkstra(int G, int *d, int *pi, int s) {
  set<int> Set;
  vector<int> Q;
  int u;
  FOR(i, G) {
    d[i] = INF;
    pi[i] = -1;
  }
  d[s] = 0;
  FOR(i, G) {
    Q.PB(i);
  }

  while (!Q.empty()) {
    vector<int>::iterator i;
    i = min_element(all(Q));
    u = *i;
    Q.erase(i);
    Set.insert(u);

    vector<pair<int, int>>::iterator it;
    for (it = graph[u].begin(); it != graph[u].end(); it++) {
      if (d[it->F] > d[u] + it->S) {
        d[it->F] = (d[u] + it->S);
        pi[it->F] = u;
      }
    }
  }
}

/*void printPath(int *prev, int start, int end, int D, ofstream &out) { 
    vector<int> path;
    unordered_map<int, int> last_seen;  // Para rastrear a última vez que vimos um vértice
    vector<int> final_path;  // Caminho final sem ciclos
    
    // Remonta o caminho inversamente, começando do fim até o início
    for (int at = end; at != -1; at = prev[at]) {
        path.push_back((at - 1) % D + 1);  // Adiciona o vértice com índice original diretamente
    }
    reverse(path.begin(), path.end());  // Inverte o caminho para imprimir do início ao fim

    // Vamos percorrer o caminho e verificar se há ciclos
    for (int i = 0; i < path.size(); ++i) {
        int current_node = path[i];

        // Se já vimos esse nó antes, significa que temos um ciclo
        if (last_seen.count(current_node)) {
            // Detecta o ciclo: trecho entre a última ocorrência e a posição atual
            int cycle_start_index = last_seen[current_node];
            
            // Imprime o ciclo detectado
            out << "Ciclo detectado: ";
            for (int j = cycle_start_index; j <= i; ++j) {
                int node = path[j];
                out << vertexIndices[node] << " (" << node << ")";
                if (j < i) {
                    out << " -> ";
                }
            }
            out << endl;

            // Remove o ciclo do caminho final
            final_path.erase(final_path.begin() + cycle_start_index, final_path.end());

            // Remove os nós do ciclo de `last_seen`, para que possam ser visitados novamente
            for (int j = cycle_start_index; j <= i; ++j) {
                last_seen.erase(path[j]);
            }
        }

        // Adiciona o nó ao caminho final e marca sua última aparição
        final_path.push_back(current_node);
        last_seen[current_node] = final_path.size() - 1;  // Atualiza com a nova posição
    }

    // Garante que o caminho final termine no vértice F (end)
    if (final_path.back() != end) {
        final_path.push_back(end);  // Adiciona o vértice F ao final, caso não esteja
    }

    // Imprime o caminho final sem ciclos
    out << "Caminho sem ciclos: ";
    for (int i = 0; i < final_path.size(); ++i) {
        int u = final_path[i];
        out << vertexIndices[u] << " (" << u << ")";  // Imprime o nome e o índice original
        
        if (i < final_path.size() - 1) {
            out << " -> ";
        }
    }
    out << endl;
}*/


void removeCycles(vector<int> &path, ofstream &out) {
    unordered_map<int, int> last_seen;  // Para rastrear a última vez que vimos um vértice
    vector<int> final_path;  // Caminho final sem ciclos

    for (int i = 0; i < path.size(); ++i) {
        int current_node = path[i];

        // Se já vimos esse nó antes, significa que temos um ciclo
        if (last_seen.count(current_node)) {
            int cycle_start_index = last_seen[current_node];

            // Imprime o ciclo detectado antes de removê-lo
            out << "Ciclo detectado: ";
            for (int j = cycle_start_index; j <= i; ++j) {
                out << " -> " << path[j];
            }
            out << endl;

            // Remove o ciclo: remove do caminho tudo entre a última ocorrência e o nó atual
            path.erase(path.begin() + cycle_start_index, path.begin() + i);

            // Chamada recursiva para continuar a remoção de ciclos no caminho restante
            removeCycles(path, out);

            return; // Após a chamada recursiva, terminamos o processamento
        }

        /*out << "Final: ";
        for (int i = 0; i < final_path.size(); ++i) {
            int u = final_path[i];
            out << vertexIndices[u] << " (" << u << ")";  // Imprime o nome e o índice original
            
            if (i < final_path.size() - 1) {
                out << " -> ";
            }
        }
        out << endl;*/
        // Se não há ciclo, adiciona o nó ao caminho final
        final_path.push_back(current_node);
        last_seen[current_node] = final_path.size() - 1;  // Atualiza a última aparição
        /*out << "Atualiza: ";
        for (int i = 0; i < final_path.size(); ++i) {
            int u = final_path[i];
            out << vertexIndices[u] << " (" << u << ")";  // Imprime o nome e o índice original
            
            if (i < final_path.size() - 1) {
                out << " -> ";
            }
        }
        out << endl;*/
    }

    // Atualiza o caminho sem ciclos
    path = final_path;
}

void removeCyclesPD(vector<int> &path, ofstream &out) {
    unordered_map<int, int> last_seen;  // Para rastrear a última vez que vimos um vértice
    vector<int> final_path;  // Caminho final sem ciclos

    for (int i = 0; i < path.size(); ++i) {
        int current_node = path[i];

        // Se já vimos esse nó antes, significa que temos um ciclo
        if (last_seen.count(current_node)) {
            int cycle_start_index = last_seen[current_node];

            // Imprime o ciclo detectado antes de removê-lo
            out << "Ciclo detectado: ";
            for (int j = cycle_start_index; j <= i; ++j) {
                out << " -> " << path[j];
            }
            out << endl;

            // Remove o ciclo mais interno: mantém o caminho até o início do ciclo
            path.erase(path.begin() + cycle_start_index + 1, path.begin() + i + 1);

            // Reinicia a verificação do caminho após remover o ciclo
            last_seen.clear();
            i = -1;  // Reinicia o loop, percorrendo o caminho desde o início
            final_path.clear();  // Limpa o caminho final para começar de novo
            continue;
        }

        // Se não há ciclo, adiciona o nó ao caminho final
        final_path.push_back(current_node);
        last_seen[current_node] = final_path.size() - 1;  // Atualiza a última aparição
    }

    // Atualiza o caminho final sem ciclos
    path = final_path;
}

void removeCyclesPD2(vector<int> &path, ofstream &out, vector<int> &path2) {
    unordered_map<int, int> last_seen;  // Para rastrear a última vez que vimos um vértice
    vector<int> final_path;  // Caminho final sem ciclos

    for (int i = 0; i < path.size(); ++i) {
        int current_node = path[i];

        // Se já vimos esse nó antes, significa que temos um ciclo
        if (last_seen.count(current_node)) {
            int cycle_start_index = last_seen[current_node];

            // Imprime o ciclo detectado antes de removê-lo
            out << "Ciclo detectado: ";
            for (int j = cycle_start_index; j <= i; ++j) {
                out << " -> " << path[j];
            }
            out << endl;

            // Remove o ciclo mais interno: mantém o caminho até o início do ciclo
            path.erase(path.begin() + cycle_start_index + 1, path.begin() + i + 1);

            // Substitui os elementos dentro do ciclo por -1 em path2
            for (int j = cycle_start_index + 1; j <= i; ++j) {
                path2[j] = -5;
            }

            // Reinicia a verificação do caminho após remover o ciclo
            last_seen.clear();
            i = -1;  // Reinicia o loop, percorrendo o caminho desde o início
            final_path.clear();  // Limpa o caminho final para começar de novo
            continue;
        }

        // Se não há ciclo, adiciona o nó ao caminho final
        final_path.push_back(current_node);
        last_seen[current_node] = final_path.size() - 1;  // Atualiza a última aparição
    }

    // Atualiza o caminho final sem ciclos
    path = final_path;
}

// Função para calcular a distância de Hamming entre duas strings
int calcularDistanciaHamming(const string &s1, const string &s2) {
    int distancia = 0;
    int tamanho_min = min(s1.size(), s2.size());

    // Calcula a distância de Hamming entre s1 e s2
    for (int i = 0; i < tamanho_min; ++i) {
        if (s1[i] != s2[i]) {
            distancia++;
        }
    }

    // Considera o comprimento excedente como diferença
    distancia += abs((int)s1.size() - (int)s2.size());

    return distancia;
}

void printPath(int *prev, int start, int end, int D, ofstream &out, string s) { 
    vector<int> path, path2;
    string caminho_resultante = "";
    int count = 0;

    // Reconstrói o caminho inversamente, começando do fim até o início
    for (int at = end; at != -1; at = prev[at]) {
        if (at == end){
          path.push_back(at);
        }
        else{
          path.push_back((at - 1) % D + 1);  // Adiciona o vértice com índice original diretamente
        }
    }
    reverse(path.begin(), path.end());  // Inverte o caminho para imprimir do início ao fim

    path2 = path;
    // Remove os ciclos do caminho
    removeCyclesPD2(path, out, path2);

    out << "Caminho: ";
    // Imprime o caminho final sem ciclos
    for (int i = 1; i < path.size()-1; ++i) {
        int u = path[i];
        out << vertexIndices[u] ;
    }
    out << endl;

    out << "Caminho '-': ";
    // Imprime o caminho com as substituições
    for (int i = 1; i < path2.size()-1; ++i) {
        int u = path2[i];
        if (u == -5){
          caminho_resultante += "-";
          count++;
          out << "-";
        }
        else{
          caminho_resultante += vertexIndices[u];
          out << vertexIndices[u] ;
        }
    }
    out << endl;
  // Calcula a distância de Hamming entre s e caminho_resultante
    int distancia_hamming_total = calcularDistanciaHamming(s, caminho_resultante);

  // Imprime os resultados
  out << "Custo(-): " << distancia_hamming_total << endl;
  out << "Custo: " << distancia_hamming_total - count << endl;

  //Separar os testes
  out << "###############################################" << endl;

}


void clearData(int T) {
  for (int i = 0; i < T; ++i) {
    graph[i].clear();
    adjList[i].clear();  // Limpa a lista de adjacência
  }
  vertexIndices.clear();
}

int main() {
  ifstream in("in");
  ofstream out("out_ciclo");

  if (!in.is_open() || !out.is_open()) {
    cerr << "Erro ao abrir arquivos." << endl;
    return 1;
  }

  int V, D, start, T, a, b, n, C;
  string s, c;
  in >> n; // número de testes
  FOR(i, n) {
    start = 0;
    in >> D; // número de vértices
    in >> s; // string que quer ler
    V = s.size();
    T = D * V + 2;
    int dist[T], prev[T];

    addVertice(0, "I"); // início
    for (int i = 1; i <= D; i++) {
      in >> c;
      addVertice(i, c);
      if (vertexIndices[i][0] == s[0])
        addAresta(0, i, 0);
      else
        addAresta(0, i, 1);
      addAresta(i + D * V - D, T - 1, 0);
    }
    addVertice(T - 1, "F"); // fim

    in >> C; // número de conexões
    FOR(i, C) {
      in >> a >> b;
      FOR(i, V - 1) {
        if (vertexIndices[b][0] == s[i + 1]) {
          addAresta(a + i * D, b + (i + 1) * D, 0);
        } else {
          addAresta(a + i * D, b + (i + 1) * D, 1);
        }
      }
    }

    // Inicia a contagem do tempo de execução para esse teste
    auto start_time = high_resolution_clock::now();

    Dijkstra(T, dist, prev, start);
    
    printPath(prev, start, T - 1, D, out, s);  // Detecta e remove ciclos durante a impressão
    //out << " Custo: " << dist[T - 1] << endl;

    // Finaliza a contagem do tempo de execução
    auto end_time = high_resolution_clock::now();
    duration<double> elapsed = end_time - start_time; // Calcula a duração em segundos

    out << "Tempo de execução: " << elapsed.count() << " segundos" << endl;

    clearData(T);
  }

  in.close();
  out.close();

  return 0;
}
