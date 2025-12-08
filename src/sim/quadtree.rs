use std::cmp::Ordering;

use macroquad::prelude::Vec2;

use crate::boid::{Boid, BoidBehavior};
use super::NeighborSearch;

#[derive(Clone, Copy)]
struct Aabb {
    min: Vec2,
    max: Vec2,
}

impl Aabb {
    fn contains(&self, p: Vec2) -> bool {
        p.x >= self.min.x && p.x <= self.max.x && p.y >= self.min.y && p.y <= self.max.y
    }

    fn intersects_circle(&self, center: Vec2, r2: f32) -> bool {
        let clamped_x = center.x.clamp(self.min.x, self.max.x);
        let clamped_y = center.y.clamp(self.min.y, self.max.y);
        let dx = center.x - clamped_x;
        let dy = center.y - clamped_y;
        dx * dx + dy * dy <= r2
    }

    fn subdivide(&self) -> [Aabb; 4] {
        let mid = (self.min + self.max) * 0.5;
        [
            // NW
            Aabb { min: Vec2::new(self.min.x, mid.y), max: Vec2::new(mid.x, self.max.y) },
            // NE
            Aabb { min: mid, max: self.max },
            // SW
            Aabb { min: self.min, max: mid },
            // SE
            Aabb { min: Vec2::new(mid.x, self.min.y), max: Vec2::new(self.max.x, mid.y) },
        ]
    }
}

struct Node {
    bounds: Aabb,
    points: Vec<(Vec2, usize)>,
    children: Option<[Box<Node>; 4]>,
    capacity: usize,
    depth: u32,
    max_depth: u32,
}

impl Node {
    fn new(bounds: Aabb, capacity: usize, depth: u32, max_depth: u32) -> Self {
        Self {
            bounds,
            points: Vec::with_capacity(capacity),
            children: None,
            capacity,
            depth,
            max_depth,
        }
    }

    fn insert(&mut self, p: Vec2, idx: usize) {
        if !self.bounds.contains(p) {
            return;
        }

        if let Some(children) = self.children.as_mut() {
            for child in children.iter_mut() {
                child.insert(p, idx);
            }
            return;
        }

        self.points.push((p, idx));

        if self.points.len() > self.capacity && self.depth < self.max_depth {
            self.subdivide();
        }
    }

    fn subdivide(&mut self) {
        let sub_bounds = self.bounds.subdivide();
        let mut children: [Box<Node>; 4] = [
            Box::new(Node::new(sub_bounds[0], self.capacity, self.depth + 1, self.max_depth)),
            Box::new(Node::new(sub_bounds[1], self.capacity, self.depth + 1, self.max_depth)),
            Box::new(Node::new(sub_bounds[2], self.capacity, self.depth + 1, self.max_depth)),
            Box::new(Node::new(sub_bounds[3], self.capacity, self.depth + 1, self.max_depth)),
        ];

        for &(p, idx) in self.points.iter() {
            for child in children.iter_mut() {
                child.insert(p, idx);
            }
        }

        self.points.clear();
        self.children = Some(children);
    }

    fn query_circle(
        &self,
        center: Vec2,
        r2: f32,
        limit: usize,
        acc: &mut Vec<(f32, usize)>,
    ) {
        if !self.bounds.intersects_circle(center, r2) {
            return;
        }

        if let Some(children) = self.children.as_ref() {
            for child in children.iter() {
                child.query_circle(center, r2, limit, acc);
            }
        } else {
            for &(p, idx) in self.points.iter() {
                let dist2 = (p - center).length_squared();
                if dist2 <= r2 && dist2 > 1e-6 {
                    if acc.len() < limit {
                        acc.push((dist2, idx));
                    } else if let Some((worst_pos, _)) = acc
                        .iter()
                        .enumerate()
                        .max_by(|(_, a), (_, b)| a.0.partial_cmp(&b.0).unwrap_or(Ordering::Equal))
                    {
                        if dist2 < acc[worst_pos].0 {
                            acc[worst_pos] = (dist2, idx);
                        }
                    }
                }
            }
        }
    }
}

pub struct QuadTreeNeighborSearch {
    root: Option<Node>,
    capacity: usize,
    max_depth: u32,
}

impl QuadTreeNeighborSearch {
    pub fn new(capacity: usize, max_depth: u32) -> Self {
        Self {
            root: None,
            capacity,
            max_depth,
        }
    }
}

impl NeighborSearch for QuadTreeNeighborSearch {
    fn rebuild(&mut self, boids: &[Boid]) {
        if boids.is_empty() {
            self.root = None;
            return;
        }

        let mut min = boids[0].pos;
        let mut max = boids[0].pos;
        for b in boids.iter().skip(1) {
            min.x = min.x.min(b.pos.x);
            min.y = min.y.min(b.pos.y);
            max.x = max.x.max(b.pos.x);
            max.y = max.y.max(b.pos.y);
        }

        let padding = 1.0;
        min -= Vec2::splat(padding);
        max += Vec2::splat(padding);

        let bounds = Aabb { min, max };
        let mut root = Node::new(bounds, self.capacity, 0, self.max_depth);

        for (idx, boid) in boids.iter().enumerate() {
            root.insert(boid.pos, idx);
        }

        self.root = Some(root);
    }

    fn neighbors(&self, boids: &[Boid], behavior: &BoidBehavior, index: usize) -> Vec<usize> {
        if let Some(root) = &self.root {
            let mut acc: Vec<(f32, usize)> = Vec::with_capacity(behavior.neighbor_limit);
            let r2 = behavior.neighbor_radius * behavior.neighbor_radius;
            let pos = boids[index].pos;
            root.query_circle(pos, r2, behavior.neighbor_limit, &mut acc);
            acc.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(Ordering::Equal));
            acc.into_iter().map(|(_, idx)| idx).collect()
        } else {
            Vec::new()
        }
    }

    fn name(&self) -> &'static str {
        "QuadTree"
    }
}

