#![feature(nll)]

extern crate petgraph;

use std::collections::HashMap;
use petgraph::{Graph, EdgeType};
use petgraph::graph::NodeIndex;
use petgraph::visit::EdgeRef;

pub struct PathIterator<'a> {
    current: Option<NodeIndex>,
    mesh: &'a NavMesh,
}

impl<'a> Iterator for PathIterator<'a> {
    type Item = NodeIndex;

    fn next(&mut self) -> Option<Self::Item> {
        let cur = self.current.take();

        if let Some(next) = cur.and_then(|cur| self.mesh.next(cur)) {
            self.current = Some(next);
        }

        cur
    }
}

pub struct NavMesh {
    target: NodeIndex,
    best: HashMap<NodeIndex, (u64, NodeIndex)>
}

impl NavMesh {
    pub fn target(&self) -> NodeIndex {
        self.target
    }

    pub fn calculate<N, D: EdgeType>(graph: Graph<N, u16, D>, target: NodeIndex) -> Self {
        let mut open = Vec::with_capacity(graph.node_count() / 8);
        open.push(target);

        let mut costs = HashMap::<NodeIndex, u64>::with_capacity(graph.node_count());
        costs.insert(target, 0);

        let mut best = HashMap::<NodeIndex, _>::with_capacity(graph.node_count());

        while let Some(idx) = open.pop() {
            let my_cost = costs[&idx];

            for edge in graph.edges(idx) {
                let cost = edge.weight();
                let target = edge.target();

                let new_cost = *cost as u64 + my_cost;

                if costs.get_mut(&target).map(|current| new_cost < *current).unwrap_or(true) {
                    best.insert(target, (new_cost, idx));
                    costs.insert(target, new_cost);

                    if !open.contains(&target) {
                        open.push(target);
                    }
                }
            }
        }

        NavMesh { target, best }
    }

    // Returns `None` if the goal is unreachable from this point.
    pub fn next(&self, from: NodeIndex) -> Option<NodeIndex> {
        self.best.get(&from).map(|&(_, node)| node)
    }

    // Returns `None` if the goal is unreachable from this point.
    pub fn cost(&self, from: NodeIndex) -> Option<u64> {
        self.best.get(&from).map(|&(cost, _)| cost)
    }

    pub fn path(&self, from: NodeIndex) -> Option<PathIterator> {
        self.next(from).map(|dir| PathIterator { current: Some(dir), mesh: self })
    }
}

#[cfg(test)]
mod tests {
    use std::collections::HashMap;
    use petgraph::{Graph, Undirected};
    use petgraph::graph::NodeIndex;
    use super::NavMesh;

    const GRID_X: usize = 5;
    const GRID_Y: usize = 4;

    fn max_y(y: isize) -> Option<isize> {
        if y >= 0 && y < GRID_Y as isize {
            Some(y)
        } else {
            None
        }
    }

    fn max_x(x: isize) -> Option<isize> {
        if x >= 0 && x < GRID_X as isize {
            Some(x)
        } else {
            None
        }
    }

    fn grid_to_graph(grid: [[bool; GRID_X]; GRID_Y]) -> (
        Graph<(), u16, Undirected>,
        HashMap<(usize, usize), NodeIndex>
    ) {
        use std::u16;
        use std::f64::consts::FRAC_1_SQRT_2;

        const ADJACENT: u16 = (u16::MAX as f64 * FRAC_1_SQRT_2) as u16;
        const DIAGONAL: u16 = u16::MAX;

        let mut nodes = HashMap::<(usize, usize), NodeIndex>::new();
        let mut out = Graph::default();

        for (y, row) in grid.iter().enumerate() {
            let elements = row.iter()
                .enumerate()
                .filter(|&(_, walkable)| *walkable)
                .map(|(i, _)| i);

            for x in elements {
                let cur_node = out.add_node(());
                nodes.insert((x, y), cur_node);

                let diffs = [
                    (-1, -1, DIAGONAL), (0, -1, ADJACENT), (1, -1, DIAGONAL),
                    (-1,  0, ADJACENT),                    (1,  0, ADJACENT),
                    (-1,  1, DIAGONAL), (0,  1, ADJACENT), (1,  1, DIAGONAL),
                ];

                for &(x_diff, y_diff, weight) in &diffs {
                    if let (Some(neighbour_x), Some(neighbour_y)) = (
                        (x as isize).checked_add(x_diff).and_then(max_x),
                        (y as isize).checked_add(y_diff).and_then(max_y),
                    ) {
                        let (neighbour_x, neighbour_y) = (neighbour_x as usize, neighbour_y as usize);

                        if grid[neighbour_y][neighbour_x] {
                            if let Some(node) = nodes.get(&(neighbour_x, neighbour_y)) {
                                out.add_edge(*node, cur_node, weight);
                            }
                        }
                    }
                }
            }
        }

        (out, nodes)
    }

    #[test]
    fn simple_path() {
        let (graph, nodes) = {
            let (x, o) = (false, true);

            grid_to_graph(
                [[o, o, o, x, o],
                 [o, x, o, x, o],
                 [o, x, o, x, o],
                 [o, x, o, o, o]]
            )
        };

        let navmesh = NavMesh::calculate(graph, nodes[&(0, 3)]);

        assert_eq!(
            navmesh.path(nodes[&(4, 0)]).unwrap().collect::<Vec<_>>(),
            [(4, 1), (4, 2), (3, 3), (2, 2), (2, 1), (1, 0), (0, 1), (0, 2), (0, 3)].iter()
                .map(|tup| nodes[tup])
                .collect::<Vec<_>>()
        );
    }

    #[test]
    fn multiple_paths() {
        let (graph, nodes) = {
            let (x, o) = (false, true);

            grid_to_graph(
                [[o, o, o, o, o],
                 [o, x, o, x, o],
                 [o, o, o, x, o],
                 [o, x, x, x, o]]
            )
        };

        let navmesh = NavMesh::calculate(graph, nodes[&(0, 3)]);

        assert_eq!(
            navmesh.path(nodes[&(4, 3)]).unwrap().collect::<Vec<_>>(),
            [(4, 2), (4, 1), (3, 0), (2, 1), (1, 2), (0, 3)].iter()
                .map(|tup| nodes[tup])
                .collect::<Vec<_>>()
        );
    }
}
