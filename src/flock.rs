use macroquad::prelude::Vec2;
use rand::Rng;

use crate::boid::{Boid, BoidBehavior};

#[derive(Clone, Copy)]
pub struct WorldBounds {
    pub w: f32,
    pub h: f32,
}

pub struct Flock {
    pub boids: Vec<Boid>,
    pub behavior: BoidBehavior,
    pub bounds: WorldBounds,
}

impl Flock {
    pub fn new_random_with_rng(
        num: usize,
        bounds: WorldBounds,
        behavior: BoidBehavior,
        rng: &mut impl Rng,
    ) -> Self {
        let mut boids = Vec::with_capacity(num);

        for _ in 0..num {
            let x = rng.random_range(behavior.edge_margin..bounds.w - behavior.edge_margin);
            let y = rng.random_range(behavior.edge_margin..bounds.h - behavior.edge_margin);
            let angle = rng.random_range(0.0..std::f32::consts::TAU);
            let speed = rng.random_range(behavior.min_speed..=behavior.max_speed);
            let dir = Vec2::from_angle(angle);

            boids.push(Boid {
                pos: Vec2::new(x, y),
                vel: dir * speed,
            });
        }

        Self { boids, behavior, bounds }
    }

    #[allow(dead_code)]
    pub fn new_random(num: usize, bounds: WorldBounds, behavior: BoidBehavior) -> Self {
        let mut rng = rand::rng();
        Self::new_random_with_rng(num, bounds, behavior, &mut rng)
    }
}
