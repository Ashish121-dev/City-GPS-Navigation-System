# City GPS Navigation System

A console-based City GPS Navigation System developed in C language using Graph Data Structures and Path Finding Algorithms.

## Overview

This project simulates a smart city navigation system where users can:
- Add places and roads
- Find shortest routes
- Search locations instantly using Hash Tables
- Detect negative cycles
- Open or close roads dynamically
- Save and load map data

The project demonstrates the practical implementation of important Data Structures and Algorithms concepts.

---

## Features

### Graph Operations
- Add new places
- Add roads between places
- Display adjacency list of the city graph

### Path Finding Algorithms
- Dijkstra Algorithm
- Bellman-Ford Algorithm
- BFS Traversal
- DFS Connectivity Check

### Hash Table Operations
- Fast place searching using hashing
- Collision handling using Linear Probing

### Advanced Features
- Negative cycle detection
- Road open/close simulation
- Save and load graph data from file
- Algorithm performance comparison

---

## Algorithms Used

| Algorithm | Purpose |
|---|---|
| Dijkstra | Shortest path for positive weights |
| Bellman-Ford | Handles negative edge weights |
| BFS | Places within K distance |
| DFS | Graph connectivity check |
| Hashing | O(1) place lookup |

---

## Technologies Used

- C Language
- Graph Data Structure
- Linked Lists
- Hash Tables
- File Handling

---

## Project Structure

```text
main.c          -> Main source code
city_map.dat    -> Saved graph data
README.md       -> Project documentation
