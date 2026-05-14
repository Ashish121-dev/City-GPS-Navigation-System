#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <limits.h>
#include <time.h>
#include <math.h>
#define MAX_PLACES      200
#define MAX_NAME_LEN    64
#define HASH_SIZE       256        
#define INF             INT_MAX
#define NEG_INF         INT_MIN
typedef struct EdgeNode {
    int   dest;
    double weight;           
    int   is_closed;         
    struct EdgeNode *next;
} EdgeNode;
typedef struct {
    char      name[MAX_NAME_LEN];
    EdgeNode *head;          
} Place;
typedef struct {
    Place  places[MAX_PLACES];
    int    count;
} Graph;
void graph_init(Graph *g) {
    g->count = 0;
}
void add_edge_directed(Graph *g, int src, int dest, double w) {
    EdgeNode *e = (EdgeNode *)malloc(sizeof(EdgeNode));
    e->dest      = dest;
    e->weight    = w;
    e->is_closed = 0;
    e->next      = g->places[src].head;
    g->places[src].head = e;
}
void add_edge(Graph *g, int src, int dest, double w) {
    add_edge_directed(g, src, dest, w);
    add_edge_directed(g, dest, src, w);
}
int add_place(Graph *g, const char *name) {
    if (g->count >= MAX_PLACES) { printf("Graph full!\n"); return -1; }
    int id = g->count++;
    strncpy(g->places[id].name, name, MAX_NAME_LEN - 1);
    g->places[id].head = NULL;
    return id;
}
void set_road_status(Graph *g, int src, int dest, int closed) {
    for (EdgeNode *e = g->places[src].head; e; e = e->next)
        if (e->dest == dest) { e->is_closed = closed; break; }
    for (EdgeNode *e = g->places[dest].head; e; e = e->next)
        if (e->dest == src) { e->is_closed = closed; break; }
}
void remove_edge(Graph *g, int src, int dest) {
    EdgeNode **pp = &g->places[src].head;
    while (*pp) {
        if ((*pp)->dest == dest) {
            EdgeNode *tmp = *pp;
            *pp = (*pp)->next;
            free(tmp);
            break;
        }
        pp = &(*pp)->next;
    }
}
typedef struct {
    char key[MAX_NAME_LEN];
    int  value;              
    int  occupied;
} HashEntry;
typedef struct {
    HashEntry table[HASH_SIZE];
    int       size;
    int       filled;
    int       collisions;
} HashTable;
unsigned int hash_func(const char *s) {
    unsigned long h = 5381;
    while (*s) h = ((h << 5) + h) + (unsigned char)(*s++);
    return (unsigned int)(h & (HASH_SIZE - 1));
}
void hash_init(HashTable *ht) {
    memset(ht->table, 0, sizeof(ht->table));
    ht->size      = HASH_SIZE;
    ht->filled    = 0;
    ht->collisions = 0;
}
void hash_insert(HashTable *ht, const char *key, int value) {
    unsigned int idx = hash_func(key);
    int probe = 0;
    while (ht->table[idx].occupied) {
        if (strcmp(ht->table[idx].key, key) == 0) {
            ht->table[idx].value = value;   
            return;
        }
        ht->collisions++;
        idx = (idx + 1) & (HASH_SIZE - 1);
        probe++;
        if (probe == HASH_SIZE) { printf("[Hash] Table full!\n"); return; }
    }
    strncpy(ht->table[idx].key, key, MAX_NAME_LEN - 1);
    ht->table[idx].value    = value;
    ht->table[idx].occupied = 1;
    ht->filled++;
}
int hash_search(HashTable *ht, const char *key) {
    unsigned int idx = hash_func(key);
    int probe = 0;
    while (ht->table[idx].occupied) {
        if (strcmp(ht->table[idx].key, key) == 0)
            return ht->table[idx].value;
        idx = (idx + 1) & (HASH_SIZE - 1);
        if (++probe == HASH_SIZE) break;
    }
    return -1;
}
void hash_print_stats(HashTable *ht) {
    printf( "  Hash Table Stats:\n" );
    printf("    Size      : %d slots\n", ht->size);
    printf("    Filled    : %d\n", ht->filled);
    printf("    Load factor: %.2f\n", (double)ht->filled / ht->size);
    printf("    Collisions: %d (resolved by linear probing)\n", ht->collisions);
}
int get_min_dist_node(double dist[], int processed[], int count) {
    double min = (double)INF;
    int min_idx = -1;
    for (int i = 0; i < count; i++) {
        if (!processed[i] && dist[i] <= min) {
            min = dist[i];
            min_idx = i;
        }
    }
    return min_idx;
}
typedef struct {
    double dist[MAX_PLACES];
    int    prev[MAX_PLACES];
    int    visited;          
    double elapsed_ms;
} ShortestPathResult;
ShortestPathResult dijkstra(Graph *g, int src) {
    ShortestPathResult res;
    res.visited = 0;
    for (int i = 0; i < g->count; i++) {
        res.dist[i] = (double)INF;
        res.prev[i] = -1;
    }
    res.dist[src] = 0.0;
    int processed[MAX_PLACES] = {0};
    clock_t t0 = clock();
    for (int count = 0; count < g->count; count++) {
        int u = get_min_dist_node(res.dist, processed, g->count);
        if (u == -1) break; 
        processed[u] = 1;
        res.visited++;
        for (EdgeNode *e = g->places[u].head; e; e = e->next) {
            if (e->is_closed) continue;
            if (e->weight < 0) continue;      
            int v = e->dest;
            double nd = res.dist[u] + e->weight;
            if (!processed[v] && res.dist[u] != (double)INF && nd < res.dist[v]) {
                res.dist[v] = nd;
                res.prev[v] = u;
            }
        }
    }
    clock_t t1 = clock();
    res.elapsed_ms = 1000.0 * (t1 - t0) / CLOCKS_PER_SEC;
    return res;
}
typedef struct {
    double dist[MAX_PLACES];
    int    prev[MAX_PLACES];
    int    neg_cycle;        
    int    edges_relaxed;
    double elapsed_ms;
} BFResult;
BFResult bellman_ford(Graph *g, int src) {
    BFResult res;
    res.neg_cycle    = 0;
    res.edges_relaxed = 0;
    for (int i = 0; i < g->count; i++) {
        res.dist[i] = (double)INF;
        res.prev[i] = -1;
    }
    res.dist[src] = 0.0;
    clock_t t0 = clock();
    for (int iter = 0; iter < g->count - 1; iter++) {
        int any = 0;
        for (int u = 0; u < g->count; u++) {
            if (res.dist[u] == (double)INF) continue;
            for (EdgeNode *e = g->places[u].head; e; e = e->next) {
                if (e->is_closed) continue;
                int v = e->dest;
                double nd = res.dist[u] + e->weight;
                if (nd < res.dist[v]) {
                    res.dist[v] = nd;
                    res.prev[v] = u;
                    res.edges_relaxed++;
                    any = 1;
                }
            }
        }
        if (!any) break;   
    }
    for (int u = 0; u < g->count; u++) {
        if (res.dist[u] == (double)INF) continue;
        for (EdgeNode *e = g->places[u].head; e; e = e->next) {
            if (e->is_closed) continue;
            if (res.dist[u] + e->weight < res.dist[e->dest]) {
                res.neg_cycle = 1;
                goto done_bf;
            }
        }
    }
done_bf:;
    clock_t t1 = clock();
    res.elapsed_ms = 1000.0 * (t1 - t0) / CLOCKS_PER_SEC;
    return res;
}
void bfs_places_within_k(Graph *g, int src, double K) {
    double dist[MAX_PLACES];
    int    visited[MAX_PLACES];
    int    queue[MAX_PLACES * 10];
    int    front = 0, back = 0;
    for (int i = 0; i < g->count; i++) {
        dist[i]    = (double)INF;
        visited[i] = 0;
    }
    dist[src] = 0.0;
    queue[back++] = src;
    visited[src] = 1;
    printf( "  Places within %.1f km of %s:\n" , K, g->places[src].name);
    int found = 0;
    while (front < back) {
        int u = queue[front++];
        for (EdgeNode *e = g->places[u].head; e; e = e->next) {
            if (e->is_closed) continue;
            int v = e->dest;
            double nd = dist[u] + e->weight;
            if (!visited[v] && nd <= K) {
                visited[v] = 1;
                dist[v]    = nd;
                queue[back++] = v;
                printf("    %-30s  %.2f km\n", g->places[v].name, nd);
                found++;
            }
        }
    }
    if (!found) printf("    (none found within %.1f km)\n", K);
}
void dfs_util(Graph *g, int u, int visited[]) {
    visited[u] = 1;
    for (EdgeNode *e = g->places[u].head; e; e = e->next)
        if (!visited[e->dest]) dfs_util(g, e->dest, visited);
}
int is_connected(Graph *g) {
    if (g->count == 0) return 1;
    int visited[MAX_PLACES] = {0};
    dfs_util(g, 0, visited);
    for (int i = 0; i < g->count; i++)
        if (!visited[i]) return 0;
    return 1;
}
void print_path(Graph *g, int prev[], int dest) {
    if (dest == -1) { printf("(no path)"); return; }
    int path[MAX_PLACES], len = 0, cur = dest;
    while (cur != -1) { path[len++] = cur; cur = prev[cur]; }
    for (int i = len - 1; i >= 0; i--) {
        printf( "%s" , g->places[path[i]].name);
        if (i > 0) printf( " -> " );
    }
}
#define SAVE_FILE "city_map.dat"
void save_graph(Graph *g) {
    FILE *f = fopen(SAVE_FILE, "w");
    if (!f) { printf("Cannot open file for saving.\n"); return; }
    fprintf(f, "%d\n", g->count);
    for (int i = 0; i < g->count; i++)
        fprintf(f, "%s\n", g->places[i].name);
    for (int i = 0; i < g->count; i++) {
        for (EdgeNode *e = g->places[i].head; e; e = e->next) {
            if (e->dest > i)  
                fprintf(f, "%d %d %.4f %d\n", i, e->dest, e->weight, e->is_closed);
        }
    }
    fprintf(f, "-1 -1 0 0\n");  
    fclose(f);
    printf( "  Graph saved to %s\n" , SAVE_FILE);
}
void load_graph(Graph *g, HashTable *ht) {
    FILE *f = fopen(SAVE_FILE, "r");
    if (!f) { printf("  No save file found (%s).\n", SAVE_FILE); return; }
    graph_init(g);
    hash_init(ht);
    int n;
    fscanf(f, "%d\n", &n);
    for (int i = 0; i < n; i++) {
        char buf[MAX_NAME_LEN];
        fgets(buf, sizeof(buf), f);
        buf[strcspn(buf, "\n")] = '\0';
        int id = add_place(g, buf);
        hash_insert(ht, buf, id);
    }
    int src, dst, closed;
    double w;
    while (fscanf(f, "%d %d %lf %d\n", &src, &dst, &w, &closed) == 4) {
        if (src == -1) break;
        add_edge(g, src, dst, w);
        for (EdgeNode *e = g->places[src].head; e; e = e->next)
            if (e->dest == dst) e->is_closed = closed;
        for (EdgeNode *e = g->places[dst].head; e; e = e->next)
            if (e->dest == src) e->is_closed = closed;
    }
    fclose(f);
    printf( "  Loaded %d places from %s\n" , g->count, SAVE_FILE);
}
void load_sample_data(Graph *g, HashTable *ht) {
    const char *places[] = {
        "ISBT Dehradun",          
        "Clock Tower",            
        "Rajpur Road",            
        "Sahastradhara",          
        "DBS Global University",  
        "Paltan Bazaar",          
        "Dehradun Railway Stn",   
        "Gandhi Park",            
        "EC Road",                
        "Mussoorie Gate",         
        "Rispana Bridge",         
        "IT Park",                
        "Doiwala",                
        "Premnagar",              
        "Raipur",                 
        "Survey Chowk",           
        "Ballupur Chowk",         
        "Kanwali Road",           
        "Haridwar Bypass",        
        "Jolly Grant Airport",    
        NULL
    };
    for (int i = 0; places[i]; i++) {
        int id = add_place(g, places[i]);
        hash_insert(ht, places[i], id);
    }
    struct { int a, b; double w; } roads[] = {
        {0,1,2.5}, {1,2,3.0}, {2,3,4.5}, {3,4,2.2}, {1,5,0.8},
        {5,6,1.2}, {6,7,1.0}, {7,8,1.5}, {8,9,8.0}, {2,8,2.0},
        {8,10,1.8},{10,11,3.2},{11,4,5.0},{4,14,3.5},{14,3,1.8},
        {1,15,1.0},{15,16,2.0},{16,17,1.5},{17,11,2.5},{0,18,6.0},
        {18,19,7.5},{6,12,12.0},{12,13,4.0},{13,0,3.5},{9,3,6.0},
        {3,17,-1.5},   
        {11,14,-0.8},  
        {-1,-1,0.0}
    };
    for (int i = 0; roads[i].a != -1; i++) {
        if (roads[i].w < 0) {
            add_edge_directed(g, roads[i].a, roads[i].b, roads[i].w);
        } else {
            add_edge(g, roads[i].a, roads[i].b, roads[i].w);
        }
    }
    printf( "  Sample city loaded: %d places, roads with some negative shortcuts.\n" , g->count);
}
void compare_algorithms(Graph *g, int src, int dst) {
    printf( "\n  --- Algorithm Comparison: %s -> %s ---\n" ,
           g->places[src].name, g->places[dst].name);
    ShortestPathResult dr = dijkstra(g, src);
    printf( "\n  [Dijkstra]  O(V^2)\n" );
    if (dr.dist[dst] == (double)INF)
        printf("    No path found.\n");
    else {
        printf("    Path  : "); print_path(g, dr.prev, dst); printf("\n");
        printf("    Dist  : %.2f km\n", dr.dist[dst]);
        printf("    Nodes visited: %d  |  Time: %.3f ms\n", dr.visited, dr.elapsed_ms);
    }
    printf("    Note: Dijkstra IGNORES negative-weight edges.\n");
    BFResult br = bellman_ford(g, src);
    printf( "\n  [Bellman-Ford]  O(V * E)\n" );
    if (br.neg_cycle)
        printf("    "  "Negative cycle detected in graph!\n" );
    else if (br.dist[dst] == (double)INF)
        printf("    No path found.\n");
    else {
        printf("    Path  : "); print_path(g, br.prev, dst); printf("\n");
        printf("    Dist  : %.2f km\n", br.dist[dst]);
        printf("    Edges relaxed: %d  |  Time: %.3f ms\n", br.edges_relaxed, br.elapsed_ms);
    }
    printf("    Note: Bellman-Ford handles negative weights correctly.\n");
    printf( "\n  --- Summary ---\n" );
    printf("    Dijkstra result : %.2f km  (%.3f ms)\n",
           dr.dist[dst] == (double)INF ? -1.0 : dr.dist[dst], dr.elapsed_ms);
    printf("    Bellman-Ford    : %.2f km  (%.3f ms)\n",
           br.dist[dst] == (double)INF ? -1.0 : br.dist[dst], br.elapsed_ms);
    if (dr.dist[dst] != (double)INF && br.dist[dst] != (double)INF) {
        if (fabs(dr.dist[dst] - br.dist[dst]) > 0.01)
            printf( "    Difference because Dijkstra missed negative edges!\n" );
        else
            printf( "    Both agree (no negative edges on this path).\n" );
    }
}
int find_place_by_name(Graph *g __attribute__((unused)), HashTable *ht, const char *name) {
    int id = hash_search(ht, name);
    if (id == -1)
        printf( "  Place \"%s\" not found in hash table.\n" , name);
    return id;
}
void print_all_places(Graph *g) {
    printf( "  All places (%d):\n" , g->count);
    for (int i = 0; i < g->count; i++)
        printf("    [%2d] %s\n", i, g->places[i].name);
}
void display_graph(Graph *g) {
    printf( "\n  --- Adjacency List ---\n" );
    for (int i = 0; i < g->count; i++) {
        printf("  [%2d] %-28s -> ", i, g->places[i].name);
        for (EdgeNode *e = g->places[i].head; e; e = e->next) {
            if (e->is_closed)
                printf( "(%s, %.1f, CLOSED)" , g->places[e->dest].name, e->weight);
            else
                printf("(%s, %.1f)", g->places[e->dest].name, e->weight);
            if (e->next) printf("  ");
        }
        printf("\n");
    }
}
void clear_input_buffer() {
    int c;
    while ((c = getchar()) != '\n' && c != EOF);
}
int read_int_safe(int *val) {
    return scanf("%d", val) == 1;
}
int main(void) {
    Graph    g;
    HashTable ht;
    graph_init(&g);
    hash_init(&ht);
    FILE *f = fopen(SAVE_FILE, "r");
    if (f) { fclose(f); load_graph(&g, &ht); }
    else    load_sample_data(&g, &ht);
    int choice;
    char name1[MAX_NAME_LEN], name2[MAX_NAME_LEN];
    while (1) {
        printf( 
               "\n+==========================================+\n"
               "|       CITY GPS NAVIGATION SYSTEM         |\n"
               "+==========================================+\n" );
        printf("  Loaded: "  "%d places"  " | Hash load: %.2f | Connected: %s\n",
               g.count,
               (double)ht.filled / ht.size,
               is_connected(&g) ?  "YES"  :  "NO" );
        printf(
               "+==========================================+\n" 
               "   1. Add Place                  6. Search Place (Hash O(1))\n"
               "   2. Add Road                   7. Places within K km (BFS)\n"
               "   3. Dijkstra Shortest Path     8. Compare Algorithms\n"
               "   4. Bellman-Ford Shortest Path 9. Display Graph\n"
               "   5. Close / Open Road         10. Hash Table Stats\n"
               "  11. Detect Negative Cycle     12. Save & Load\n"
               "  13. List All Places            0. Exit\n"
                "==========================================+\n" 
               "  > ");
        if (!read_int_safe(&choice)) { clear_input_buffer(); continue; }
        clear_input_buffer();
        if (choice == 0) {
            printf( "  Goodbye!\n" );
            break;
        } else if (choice == 1) {
            printf("  Enter place name: ");
            fgets(name1, sizeof(name1), stdin);
            name1[strcspn(name1, "\n")] = '\0';
            if (hash_search(&ht, name1) != -1) {
                printf( "  Place already exists.\n" );
            } else {
                int id = add_place(&g, name1);
                hash_insert(&ht, name1, id);
                printf( "  Added [%d] %s\n" , id, name1);
            }
        } else if (choice == 2) {
            print_all_places(&g);
            int src, dst; double w;
            printf("  From (node ID): "); read_int_safe(&src); clear_input_buffer();
            printf("  To   (node ID): "); read_int_safe(&dst); clear_input_buffer();
            if (src < 0 || src >= g.count || dst < 0 || dst >= g.count) {
                printf( "  Invalid node IDs.\n" ); continue;
            }
            printf("  Distance (km, can be negative for bonus): ");
            scanf("%lf", &w); clear_input_buffer();
            add_edge(&g, src, dst, w);
            printf( "  Road added: %s <-> %s (%.2f km)\n" ,
                   g.places[src].name, g.places[dst].name, w);
        } else if (choice == 3) {
            printf("  From place name: "); fgets(name1, sizeof(name1), stdin); name1[strcspn(name1,"\n")]='\0';
            printf("  To   place name: "); fgets(name2, sizeof(name2), stdin); name2[strcspn(name2,"\n")]='\0';
            int src = find_place_by_name(&g, &ht, name1);
            int dst = find_place_by_name(&g, &ht, name2);
            if (src == -1 || dst == -1) continue;
            ShortestPathResult res = dijkstra(&g, src);
            printf( "\n  [Dijkstra] %s -> %s\n" , name1, name2);
            if (res.dist[dst] == (double)INF)
                printf("  No path found (graph may be disconnected).\n");
            else {
                printf("  Path    : "); print_path(&g, res.prev, dst); printf("\n");
                printf("  Distance: %.2f km\n", res.dist[dst]);
                printf("  Nodes visited: %d  |  Time: %.3f ms\n", res.visited, res.elapsed_ms);
                printf("  Complexity: O(V^2) - V=%d, E varies\n", g.count);
            }
        } else if (choice == 4) {
            printf("  From place name: "); fgets(name1, sizeof(name1), stdin); name1[strcspn(name1,"\n")]='\0';
            printf("  To   place name: "); fgets(name2, sizeof(name2), stdin); name2[strcspn(name2,"\n")]='\0';
            int src = find_place_by_name(&g, &ht, name1);
            int dst = find_place_by_name(&g, &ht, name2);
            if (src == -1 || dst == -1) continue;
            BFResult res = bellman_ford(&g, src);
            printf( "\n  [Bellman-Ford] %s -> %s\n" , name1, name2);
            if (res.neg_cycle)
                printf( "  Negative cycle detected! Distances are unreliable.\n" );
            else if (res.dist[dst] == (double)INF)
                printf("  No path found.\n");
            else {
                printf("  Path    : "); print_path(&g, res.prev, dst); printf("\n");
                printf("  Distance: %.2f km\n", res.dist[dst]);
                printf("  Edges relaxed: %d  |  Time: %.3f ms\n", res.edges_relaxed, res.elapsed_ms);
                printf("  Complexity: O(V*E) - V=%d\n", g.count);
            }
        } else if (choice == 5) {
            print_all_places(&g);
            int src, dst, status;
            printf("  From (node ID): "); read_int_safe(&src); clear_input_buffer();
            printf("  To   (node ID): "); read_int_safe(&dst); clear_input_buffer();
            if (src < 0 || src >= g.count || dst < 0 || dst >= g.count) {
                printf( "  Invalid IDs.\n" ); continue;
            }
            printf("  0 = Open, 1 = Close: "); read_int_safe(&status); clear_input_buffer();
            set_road_status(&g, src, dst, status);
            printf(status ?  "  Road closed: %s <-> %s\n"  :  "  Road opened: %s <-> %s\n" ,
                   g.places[src].name, g.places[dst].name);
        } else if (choice == 6) {
            printf("  Enter place name to search: ");
            fgets(name1, sizeof(name1), stdin); name1[strcspn(name1,"\n")]='\0';
            int id = hash_search(&ht, name1);
            if (id == -1)
                printf( "  \"%s\" not found.\n" , name1);
            else
                printf( "  Found: [%d] %s  (O(1) hash lookup)\n" , id, g.places[id].name);
        } else if (choice == 7) {
            printf("  From place name: "); fgets(name1, sizeof(name1), stdin); name1[strcspn(name1,"\n")]='\0';
            int src = find_place_by_name(&g, &ht, name1);
            if (src == -1) continue;
            double K; printf("  Max distance K (km): "); scanf("%lf", &K); clear_input_buffer();
            bfs_places_within_k(&g, src, K);
        } else if (choice == 8) {
            printf("  From place name: "); fgets(name1, sizeof(name1), stdin); name1[strcspn(name1,"\n")]='\0';
            printf("  To   place name: "); fgets(name2, sizeof(name2), stdin); name2[strcspn(name2,"\n")]='\0';
            int src = find_place_by_name(&g, &ht, name1);
            int dst = find_place_by_name(&g, &ht, name2);
            if (src == -1 || dst == -1) continue;
            compare_algorithms(&g, src, dst);
        } else if (choice == 9) {
            display_graph(&g);
        } else if (choice == 10) {
            hash_print_stats(&ht);
        } else if (choice == 11) {
            printf("  Running Bellman-Ford from node 0 to detect negative cycles...\n");
            BFResult res = bellman_ford(&g, 0);
            if (res.neg_cycle)
                printf( "  !  Negative cycle DETECTED in graph!\n" );
            else
                printf( "  v  No negative cycle found.\n" );
        } else if (choice == 12) {
            int sub;
            printf("  1 = Save  2 = Reload: "); read_int_safe(&sub); clear_input_buffer();
            if (sub == 1) save_graph(&g);
            else { load_graph(&g, &ht); }
        } else if (choice == 13) {
            print_all_places(&g);
        } else {
            printf( "  Invalid option.\n" );
        }
    }
    return 0;
}