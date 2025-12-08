use std::cmp::Ordering;

use crate::boid::{Boid, BoidBehavior};
use super::NeighborSearch;

pub struct BruteForceNeighborSearch;

impl NeighborSearch for BruteForceNeighborSearch {
    fn rebuild(&mut self, _boids: &[Boid]) {
        // Nothing to rebuild for brute force.
    }

    fn neighbors(&self, boids: &[Boid], behavior: &BoidBehavior, index: usize) -> Vec<usize> {
        let visual_r2 = behavior.neighbor_radius * behavior.neighbor_radius;
        let mut candidates: Vec<(f32, usize)> = Vec::new();

        for (j, other) in boids.iter().enumerate() {
            if j == index {
                continue;
            }
            let offset = other.pos - boids[index].pos;
            let dist2 = offset.length_squared();
            if dist2 < visual_r2 && dist2 > 1e-6 {
                candidates.push((dist2, j));
            }
        }

        candidates.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(Ordering::Equal));

        let limit = behavior.neighbor_limit;
        candidates
            .into_iter()
            .take(limit)
            .map(|(_, idx)| idx)
            .collect()
    }

    fn name(&self) -> &'static str {
        "BruteForce"
    }
}

