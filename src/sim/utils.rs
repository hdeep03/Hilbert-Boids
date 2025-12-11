use macroquad::prelude::{Vec2, Mat2};
use crate::boid::{Boid, BoidBehavior};
use crate::flock::WorldBounds;
use rand::Rng;
use rand_distr::{Distribution, Normal};

pub fn keep_within_bounds(boid: &mut Boid, bounds: &WorldBounds, behavior: &BoidBehavior) {
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

pub fn limit_speed(boid: &mut Boid, behavior: &BoidBehavior) {
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
pub fn random_vel_change(boid: &mut Boid, behavior: &BoidBehavior, rng: &mut impl Rng) {
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