use std::time::Instant;
use macroquad::prelude::*;

mod boid;
mod flock;
mod sim;

use boid::{Boid, BoidBehavior};
use flock::{Flock, WorldBounds};
use sim::{BoidSim, Sim, HilbertNeighborSearch};

const BOID_LINE_WIDTH: f32 = 1.0;
const MSAA_SAMPLE_COUNT: i32 = 1;

fn hsv_to_rgb(h: f32, s: f32, v: f32) -> Color {
    let h = h.rem_euclid(1.0) * 6.0;
    let i = h.floor() as i32;
    let f = h - i as f32;
    let p = v * (1.0 - s);
    let q = v * (1.0 - s * f);
    let t = v * (1.0 - s * (1.0 - f));

    let (r, g, b) = match i % 6 {
        0 => (v, t, p),
        1 => (q, v, p),
        2 => (p, v, t),
        3 => (p, q, v),
        4 => (t, p, v),
        _ => (v, p, q),
    };

    Color::new(r, g, b, 1.0)
}

fn heading_color(forward: Vec2) -> Color {
    let angle = forward.y.atan2(forward.x);
    let hue = (angle / std::f32::consts::TAU).rem_euclid(1.0);
    hsv_to_rgb(hue, 0.8, 1.0)
}

#[allow(unused)]
fn _noop_spatial_hash() {
    // Keeps SpatialHashNeighborSearch::new referenced to silence dead_code warnings when unused.
    let _ = sim::SpatialHashNeighborSearch::new(1.0);
}

fn window_conf() -> Conf {
    Conf {
        window_title: "Boids".to_owned(),
        window_width: 1920,
        window_height: 1080,
        sample_count: MSAA_SAMPLE_COUNT,
        fullscreen: true,
        high_dpi: true,
        ..Default::default()
    }
}

fn draw_boid(boid: &Boid, scale: f32) {
    let pos = boid.pos;
    let vel = boid.vel;
    let speed2 = vel.length_squared();
    if speed2 < 1e-6 {
        return; // no direction → don't draw
    }

    let forward = vel.normalize();
    let right = Vec2::new(forward.y, -forward.x); // perpendicular

    let width = BOID_LINE_WIDTH;

    // Shape parameters
    let tip_len = 6.0 * scale;
    let base_len = 2.5 * scale;
    let wing = 3.0 * scale;

    // Points in world space
    let tip   = pos + forward * tip_len;
    let left  = pos - forward * base_len - right * wing;
    let right_p = pos - forward * base_len + right * wing;

    let color = heading_color(forward);

    // Draw triangle / dart
    draw_line(tip.x, tip.y, left.x, left.y, width, color);
    draw_line(left.x, left.y, pos.x, pos.y, width, color);
    draw_line(pos.x, pos.y, right_p.x, right_p.y, width, color);
    draw_line(right_p.x, right_p.y, tip.x, tip.y, width, color);
}


#[macroquad::main(window_conf)]
async fn main() {
    // World matches your window size
    let bounds = WorldBounds {
        w: screen_width(),
        h: screen_height(),
    };

    let num_boids = 100000;

    // Use default boid behavior for now
    let behavior: BoidBehavior = BoidBehavior::default();

    // Create a random flock
    let flock = Flock::new_random(num_boids, bounds, behavior);

    // Use our pluggable sim with selected neighbor search
    // let mut sim = Sim::new(flock, Box::new(QuadTreeNeighborSearch::new(8, 6)));
    let mut sim = Sim::new(flock, Box::new(HilbertNeighborSearch::new(16)));

    loop {
        let dt = get_frame_time();
        let start = Instant::now();
        sim.step(dt);
        let end = Instant::now();
        let duration = end.duration_since(start);
        // println!("Time taken: {:?}", duration);
        clear_background(BLACK);

        // Draw all boids
        for b in sim.boids() {
            draw_boid(b, 0.1);
        }

        // Small overlay text so you know which sim this is
        let ups = 1.0 / dt;
        draw_text(
            format!("Sim ({}) boids: {} engine_time: {:.2}ms", sim.algo_name(), num_boids,  (duration.as_micros() as f32)/1000.0).as_str(),
            20.0,
            40.0,
            32.0,
            WHITE,
        );

        next_frame().await;
    }
}
