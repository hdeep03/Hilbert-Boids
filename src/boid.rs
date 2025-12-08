use macroquad::prelude::*;

#[derive(Clone, Copy, Debug)]
pub struct Boid {
    pub pos: Vec2,
    pub vel: Vec2,
}

#[derive(Clone, Copy, Debug)]
pub struct BoidBehavior {
    pub neighbor_radius: f32,
    pub avoidance_radius: f32,
    pub neighbor_limit: usize,
    pub max_speed: f32,
    pub min_speed: f32,
    pub fov_deg: f32,
    pub weight_separation: f32,
    pub weight_alignment: f32,
    pub weight_cohesion: f32,

    pub edge_margin: f32,
    pub edge_turn_factor: f32,

    pub max_rand_rotate: f32
}

impl Default for BoidBehavior {
    fn default() -> Self {
        Self {
            neighbor_radius: 150.0,
            avoidance_radius: 100.0,
            neighbor_limit: 64,
            max_speed: 100.0,
            min_speed: 10.0,
            fov_deg: 360.0,
            weight_separation: 10.0,
            weight_alignment: 0.9,
            weight_cohesion: 0.5,
            edge_margin: 50.0,
            edge_turn_factor: 0.2,
            max_rand_rotate: 0.15,
        }
    }
}
