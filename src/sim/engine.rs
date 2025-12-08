use macroquad::prelude::{Mat2, Vec2};
use rayon::prelude::*;
use rand::Rng;
use rand_distr::{Distribution, Normal};
use std::time::Instant;

use crate::boid::{Boid, BoidBehavior};
use crate::flock::{Flock, WorldBounds};
use super::{BoidSim, NeighborSearch, BruteForceNeighborSearch};

pub struct Sim {
    flock: Flock,
    neighbors: Box<dyn NeighborSearch>,
}

impl Sim {
    pub fn new(flock: Flock, neighbors: Box<dyn NeighborSearch>) -> Self {
        Self { flock, neighbors }
    }

    #[allow(dead_code)]
    /// Convenience helper for the default brute-force neighbor search.
    pub fn with_brute_force(flock: Flock) -> Self {
        Self::new(flock, Box::new(BruteForceNeighborSearch))
    }

    pub fn algo_name(&self) -> &'static str {
        self.neighbors.name()
    }

    fn keep_within_bounds(boid: &mut Boid, bounds: &WorldBounds, behavior: &BoidBehavior) {
        let pixel_margin = behavior.edge_margin;

        if boid.pos.x < pixel_margin {
            boid.vel = boid
                .vel
                .lerp(Vec2::new(behavior.min_speed, 0.0), behavior.edge_turn_factor);
        } else if boid.pos.x > bounds.w - pixel_margin {
            boid.vel = boid
                .vel
                .lerp(Vec2::new(-behavior.min_speed, 0.0), behavior.edge_turn_factor);
        }

        if boid.pos.y < pixel_margin {
            boid.vel = boid
                .vel
                .lerp(Vec2::new(0.0, behavior.min_speed), behavior.edge_turn_factor);
        } else if boid.pos.y > bounds.h - pixel_margin {
            boid.vel = boid
                .vel
                .lerp(Vec2::new(0.0, -behavior.min_speed), behavior.edge_turn_factor);
        }
    }

    fn limit_speed(boid: &mut Boid, behavior: &BoidBehavior) {
        let speed = boid.vel.length();
        if speed > behavior.max_speed {
            boid.vel = boid.vel / speed * behavior.max_speed;
        } else if speed < behavior.min_speed {
            if speed > 1e-6 {
                boid.vel = boid.vel / speed * behavior.min_speed;
            } else {
                // If totally stuck, give it a nudge
                boid.vel = Vec2::new(behavior.min_speed, 0.0);
            }
        }
    }

    fn random_vel_change(boid: &mut Boid, behavior: &BoidBehavior, rng: &mut impl Rng) {
        if behavior.max_rand_rotate <= 0.0 {
            return;
        }
        let stdev = behavior.max_rand_rotate / 3.0;
        let normal = Normal::new(0.0f32, stdev).unwrap();
        let angle = normal.sample(rng);
        let rot = Mat2::from_cols_array(&[
            angle.cos(), angle.sin(),
            -angle.sin(), angle.cos(),
        ]);
        boid.vel = rot * boid.vel;
    }
}

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

        // 1) compute "rule" contributions based on previous frame state
        let boids_snapshot = self.flock.boids.clone(); // so we don't read/write same frame
        let mut accels = vec![Vec2::ZERO; n];

        let t_neighbor_start = Instant::now();
        self.neighbors.rebuild(&boids_snapshot);
        let t_neighbor_end = Instant::now();

        let t_neighbors_start = Instant::now();
        accels
            .par_iter_mut()
            .enumerate()
            .for_each(|(i, accel)| {
                let me = boids_snapshot[i];

                let mut center = Vec2::ZERO;      // for cohesion
                let mut avg_vel = Vec2::ZERO;     // for alignment
                let mut separation = Vec2::ZERO;  // for separation
                let mut separation_count = 0;
                let mut total_weight = 0.0;       // weighted neighbor count

                for j in self.neighbors.neighbors(&boids_snapshot, behavior, i) {
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

                    // --- Distance weight for cohesion + alignment ---
                    // dist = 0                 -> weight ~ 1
                    // dist = neighbor_radius   -> weight ~ 0
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
        let t_neighbors_end = Instant::now();

        // 2) integrate velocities/positions & apply extra behaviors
        let mut rng = rand::rng();
        let center = Vec2::new(bounds.w * 0.5, bounds.h * 0.5);

        for (idx, boid) in self.flock.boids.iter_mut().enumerate() {
            boid.vel += accels[idx] * dt;

            Self::random_vel_change(boid, behavior, &mut rng);

            // Gentle bias toward center, stronger near walls.
            let dist_left = boid.pos.x;
            let dist_right = bounds.w - boid.pos.x;
            let dist_bottom = boid.pos.y;
            let dist_top = bounds.h - boid.pos.y;
            let min_edge = dist_left.min(dist_right).min(dist_bottom).min(dist_top).max(1.0);
            let to_center = center - boid.pos;
            if to_center.length_squared() > 1e-6 {
                let bias_dir = to_center.normalize();
                // Stronger pull: scale harder as we approach edges.
                // Base term (edge_margin / min_edge) plus an extra factor.
                let bias_mag = (behavior.edge_margin / min_edge) * behavior.edge_turn_factor * 3.0;
                boid.vel += bias_dir * bias_mag * dt;
            }

            // Limit speed
            Self::limit_speed(boid, behavior);

            // Integrate position
            boid.pos += boid.vel * dt;

            // Keep within bounds
            Self::keep_within_bounds(boid, bounds, behavior);
        }

        let _t_neighbor_total = (t_neighbor_end - t_neighbor_start) + (t_neighbors_end - t_neighbors_start);
        // println!(
        //     "neighbor search: rebuild {:?}, queries {:?}, total {:?}",
        //     t_neighbor_end - t_neighbor_start,
        //     t_neighbors_end - t_neighbors_start,
        //     t_neighbor_total
        // );
    }

    fn boids(&self) -> &[Boid] {
        &self.flock.boids
    }
}

