use crate::boid::{Boid, BoidBehavior};

pub trait BoidSim {
    fn step(&mut self, dt: f32);
    fn boids(&self) -> &[Boid];
}

/// A pluggable neighbor query that can be swapped without touching boid update logic.
pub trait NeighborSearch: Send + Sync {
    /// Rebuild internal structures based on the current boid positions.
    fn rebuild(&mut self, boids: &[Boid]);

    /// Return indices of boids considered neighbors of `index` given the behavior settings.
    fn neighbors(&self, boids: &[Boid], behavior: &BoidBehavior, index: usize) -> Vec<usize>;

    /// Human-readable name for display/debugging.
    fn name(&self) -> &'static str;
}

// mod hilbert;
// mod grid;
mod engine;
mod brute_force;
mod spatial_hashing;
mod quadtree;
mod hilbert;
mod hilbert_dual;
// pub use hilbert::HilbertSim;
// pub use grid::GridSim;
pub use engine::Sim;
pub use brute_force::BruteForceNeighborSearch;
#[allow(unused_imports)]
pub use spatial_hashing::SpatialHashNeighborSearch;
#[allow(unused_imports)]
pub use hilbert::HilbertNeighborSearch;
#[allow(unused_imports)]
pub use quadtree::QuadTreeNeighborSearch;
#[allow(unused_imports)]
pub use hilbert_dual::HilbertDualNeighborSearch;

#[allow(dead_code)]
fn _noop_dual_hilbert() {
    // Reference to avoid dead_code lint when unused.
    let _ = hilbert_dual::HilbertDualNeighborSearch::new(8);
}