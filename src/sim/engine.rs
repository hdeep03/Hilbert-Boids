use macroquad::prelude:: Vec2;
use rayon::prelude::*;
use rand::rngs::StdRng;
use std::time::Instant;
use crate::boid::Boid;
use crate::flock::Flock;
use crate::sim::utils::{keep_within_bounds, limit_speed, random_vel_change};
use super::{BoidSim, NeighborSearch, BruteForceNeighborSearch};

pub struct Sim {
    flock: Flock,
    neighbors: Box<dyn NeighborSearch>,
    rng: StdRng,
}

// pub struct SeqSim {
//     flock: Flock,
//     rng: StdRng,
// }

impl Sim {
    pub fn new(flock: Flock, neighbors: Box<dyn NeighborSearch>, rng: StdRng) -> Self {
        Self { flock, neighbors, rng }
    }

    #[allow(dead_code)]
    /// Convenience helper for the default brute-force neighbor search.
    pub fn with_brute_force(flock: Flock, rng: StdRng) -> Self {
        Self::new(flock, Box::new(BruteForceNeighborSearch), rng)
    }

    pub fn algo_name(&self) -> &'static str {
        self.neighbors.name()
    }
}

// impl SeqSim {
//     pub fn new(flock: Flock, rng: StdRng) -> Self {
//         Self { flock, rng }
//     }
// }

impl BoidSim for Sim {
    fn step(&mut self, dt: f32) {
        let n = self.flock.boids.len();
        if n == 0 {
            return;
        }
        let bounds   = &self.flock.bounds;
        let behavior = &self.flock.behavior;

        let avoid_r2    = behavior.avoidance_radius * behavior.avoidance_radius;
        let sight_angle = behavior.fov_deg.to_radians();

        let mut accels = vec![Vec2::ZERO; n];
        {
            let boids_snapshot = &self.flock.boids;
            self.neighbors.rebuild(boids_snapshot);
            accels
                .par_iter_mut()
                .enumerate()
                .for_each(|(i, accel)| {
                    let me = boids_snapshot[i];

                    let mut center = Vec2::ZERO;
                    let mut avg_vel = Vec2::ZERO;
                    let mut separation = Vec2::ZERO;
                    let mut separation_count = 0;
                    let mut total_weight = 0.0;

                    for j in self.neighbors.neighbors(boids_snapshot, behavior, i) {
                        let other = boids_snapshot[j];
                        let offset = other.pos - me.pos;
                        let dist2 = offset.length_squared();

                        if dist2 < 1e-6 {
                            continue;
                        }

                        let dist = dist2.sqrt();

                        if dist2 < avoid_r2 {
                            let dir_away = me.pos - other.pos;
                            let dir_away_len = dir_away.length();
                            separation += dir_away / dir_away_len;
                            separation_count += 1;
                        }

                        let angle = me.vel.angle_between(offset);
                        if angle < -sight_angle * 0.5 || angle > sight_angle * 0.5 {
                            continue;
                        }

                        let t = dist / behavior.neighbor_radius;
                        let w = if t < 1.0 { 1.0 - t } else { 0.0 };
                        if w <= 0.0 {
                            continue;
                        }

                        center += other.pos * w;
                        avg_vel += other.vel * w;
                        total_weight += w;
                    }

                    if separation_count > 0 {
                        separation /= separation_count as f32;
                    }

                    let mut a = Vec2::ZERO;

                    if total_weight > 0.0 {
                        let inv = 1.0 / total_weight;

                        // --- fly_towards_center (cohesion) ---
                        center *= inv;
                        a += (center - me.pos) * behavior.weight_cohesion;

                        // --- match_velocities (alignment) ---
                        avg_vel *= inv;
                        a += (avg_vel - me.vel) * behavior.weight_alignment;
                    }

                    // --- avoid_other_boids (separation) ---
                    a += separation * behavior.weight_separation;

                    *accel = a;
                });
        }

        let rng = &mut self.rng;
        let center = Vec2::new(bounds.w * 0.5, bounds.h * 0.5);

        for (idx, boid) in self.flock.boids.iter_mut().enumerate() {
            boid.vel += accels[idx] * dt;
            random_vel_change(boid, behavior, rng);
            let dist_left = boid.pos.x;
            let dist_right = bounds.w - boid.pos.x;
            let dist_bottom = boid.pos.y;
            let dist_top = bounds.h - boid.pos.y;
            let min_edge = dist_left.min(dist_right).min(dist_bottom).min(dist_top).max(1.0);
            let to_center = center - boid.pos;
            if to_center.length_squared() > 1e-6 {
                let bias_dir = to_center.normalize();
                let bias_mag = (behavior.edge_margin / min_edge) * behavior.edge_turn_factor * 3.0;
                boid.vel += bias_dir * bias_mag * dt;
            }
            limit_speed(boid, behavior);
            boid.pos += boid.vel * dt;
            keep_within_bounds(boid, bounds, behavior);
        }

    }

    fn boids(&self) -> &[Boid] {
        &self.flock.boids
    }
}


// impl BoidSim for SeqSim {
//     fn step(&mut self, dt: f32) {
//         let n = self.flock.boids.len();
//         if n == 0 {
//             return;
//         }
//         let bounds   = &self.flock.bounds;
//         let behavior = &self.flock.behavior;
//     }

//     fn boids(&self) -> &[Boid] {
//         &self.flock.boids
//     }
// }