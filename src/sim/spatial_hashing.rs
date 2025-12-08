#![allow(dead_code)]

use std::cmp::Ordering;
use std::collections::HashMap;

use macroquad::prelude::Vec2;
use crate::boid::{Boid, BoidBehavior};
use super::NeighborSearch;

/// Spatial hashing–based neighbor search.
/// 
/// Assumes a 2D world; `cell_size` should be on the order of `neighbor_radius`
/// (often `cell_size = neighbor_radius` or slightly smaller).
pub struct SpatialHashNeighborSearch {
    pub cell_size: f32,
    grid: HashMap<(i32, i32), Vec<usize>>,
}

#[allow(dead_code)]
impl SpatialHashNeighborSearch {
    #[allow(dead_code, unused)]
    pub fn new(cell_size: f32) -> Self {
        Self {
            cell_size,
            grid: HashMap::new(),
        }
    }

    #[inline]
    fn cell_of(&self, pos: Vec2) -> (i32, i32) {
        let cx = (pos.x / self.cell_size).floor() as i32;
        let cy = (pos.y / self.cell_size).floor() as i32;
        (cx, cy)
    }
}

impl NeighborSearch for SpatialHashNeighborSearch {
    fn rebuild(&mut self, boids: &[Boid]) {
        self.grid.clear();

        for (i, boid) in boids.iter().enumerate() {
            let cell = self.cell_of(boid.pos);
            self.grid.entry(cell).or_default().push(i);
        }
    }

    fn neighbors(&self, boids: &[Boid], behavior: &BoidBehavior, index: usize) -> Vec<usize> {
        let visual_r2 = behavior.neighbor_radius * behavior.neighbor_radius;
        let pos_i = boids[index].pos;

        let (cx, cy) = self.cell_of(pos_i);

        // How many cells around us to search in each direction.
        // If cell_size <= neighbor_radius, 1 cell in each direction is enough.
        let cell_search_radius =
            (behavior.neighbor_radius / self.cell_size).ceil() as i32;

        let limit = behavior.neighbor_limit;
        let mut candidates: Vec<(f32, usize)> = Vec::with_capacity(limit.saturating_mul(2));
        let mut worst = 0.0f32;

        let consider = |dist2: f32, j: usize, candidates: &mut Vec<(f32, usize)>, worst: &mut f32| {
            if candidates.len() < limit {
                candidates.push((dist2, j));
                if dist2 > *worst {
                    *worst = dist2;
                }
            } else if dist2 < *worst {
                // Replace the current worst with this closer neighbor.
                if let Some(pos) = candidates.iter().position(|(d, _)| *d == *worst) {
                    candidates[pos] = (dist2, j);
                    *worst = candidates.iter().fold(0.0, |acc, (d, _)| acc.max(*d));
                }
            }
        };

        // Visit cells in expanding "rings" so closer cells are considered first.
        for r in 0..=cell_search_radius {
            if r == 0 {
                if let Some(indices) = self.grid.get(&(cx, cy)) {
                    for &j in indices {
                        if j == index {
                            continue;
                        }
                        let dist2 = (boids[j].pos - pos_i).length_squared();
                        if dist2 < visual_r2 && dist2 > 1e-6 {
                            consider(dist2, j, &mut candidates, &mut worst);
                        }
                    }
                }
                continue;
            }

            // Top and bottom rows of the ring
            for dx in -r..=r {
                for &dy in [-r, r].iter() {
                    let key = (cx + dx, cy + dy);
                    if let Some(indices) = self.grid.get(&key) {
                        for &j in indices {
                            if j == index {
                                continue;
                            }
                            let dist2 = (boids[j].pos - pos_i).length_squared();
                            if dist2 < visual_r2 && dist2 > 1e-6 {
                                consider(dist2, j, &mut candidates, &mut worst);
                            }
                        }
                    }
                }
            }

            // Left and right columns of the ring (excluding corners already handled)
            for dy in (-r + 1)..=(r - 1) {
                for &dx in [-r, r].iter() {
                    let key = (cx + dx, cy + dy);
                    if let Some(indices) = self.grid.get(&key) {
                        for &j in indices {
                            if j == index {
                                continue;
                            }
                            let dist2 = (boids[j].pos - pos_i).length_squared();
                            if dist2 < visual_r2 && dist2 > 1e-6 {
                                consider(dist2, j, &mut candidates, &mut worst);
                            }
                        }
                    }
                }
            }
        }

        candidates.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(Ordering::Equal));
        candidates.into_iter().map(|(_, idx)| idx).collect()
    }

    fn name(&self) -> &'static str {
        "SpatialHash"
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn can_construct() {
        let _ = SpatialHashNeighborSearch::new(1.0);
    }
}
