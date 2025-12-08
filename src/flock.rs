use macroquad::prelude::Vec2;
use rand::Rng;

use crate::boid::{Boid, BoidBehavior};

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
    pub fn new_random(num: usize, bounds: WorldBounds, behavior: BoidBehavior) -> Self {
        let mut rng = rand::rng();
        let mut boids = Vec::with_capacity(num);

        for _ in 0..num {
            let x = rng.random_range(0.0..bounds.w);
            let y = rng.random_range(0.0..bounds.h);
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
}
