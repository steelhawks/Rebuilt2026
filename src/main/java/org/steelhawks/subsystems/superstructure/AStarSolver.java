package org.steelhawks.subsystems.superstructure;

import java.util.*;

// THE MOST BEAUTIFUL PIECE OF CODE I HAVE EVER WRITTEN

public class AStarSolver<N> {
    public interface Heuristic<N> {
        double estimate(N current, N goal);
    }

    public interface Neighbors<N> {
        List<Edge<N>> getNeighbors(N node);
    }

//    public record Edge<N>(N neighbor, double cost) {}

     public static class Edge<N> {
       public final N neighbor;
       public final double cost;

       public Edge(N neighbor, double cost) {
           this.neighbor = neighbor;
           this.cost = cost;
       }
     }


    public List<N> solve(N start, N goal, Heuristic<N> heuristic, Neighbors<N> neighbors) {
        Map<N, Double> gScore = new HashMap<>();
        Map<N, N> cameFrom = new HashMap<>();
        PriorityQueue<NodeRecord<N>> openSet = new PriorityQueue<>(Comparator.comparingDouble(nr -> nr.fScore));

        gScore.put(start, 0.0);
        openSet.add(new NodeRecord<>(start, heuristic.estimate(start, goal)));

        while (!openSet.isEmpty()) {
            NodeRecord<N> currentRecord = openSet.poll();
            N current = currentRecord.node;

            if (current.equals(goal)) {
                return reconstructPath(cameFrom, current);
            }

            for (Edge<N> edge : neighbors.getNeighbors(current)) {
                double tentativeGScore = gScore.get(current) + edge.cost;
                if (tentativeGScore < gScore.getOrDefault(edge.neighbor, Double.POSITIVE_INFINITY)) {
                    cameFrom.put(edge.neighbor, current);
                    gScore.put(edge.neighbor, tentativeGScore);
                    double fScore = tentativeGScore + heuristic.estimate(edge.neighbor, goal);
                    openSet.add(new NodeRecord<>(edge.neighbor, fScore));
                }
            }
        }

        return null;
    }

    private List<N> reconstructPath(Map<N, N> cameFrom, N current) {
        List<N> path = new ArrayList<>();
        path.add(current);
        while (cameFrom.containsKey(current)) {
            current = cameFrom.get(current);
            path.add(current);
        }

        Collections.reverse(path);
        return path;
    }

    private static class NodeRecord<N> {
        private final N node;
        private final double fScore;

        public NodeRecord(N node, double fScore) {
            this.node = node;
            this.fScore = fScore;
        }

    }


//    private record NodeRecord<N>(N node, double fScore) {}

}
