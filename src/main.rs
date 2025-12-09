use std::env;
use std::time::Instant;
use std::collections::VecDeque;
use macroquad::prelude::*;
use ::rand::{rngs::StdRng, SeedableRng};

mod boid;
mod flock;
mod sim;

use boid::{Boid, BoidBehavior};
use flock::{Flock, WorldBounds};
use sim::{BoidSim, Sim};
#[allow(unused_imports)]
use sim::{HilbertRotatedNeighborSearch, QuadTreeNeighborSearch, HilbertNeighborSearch, BruteForceNeighborSearch};


#[derive(Clone, Copy)]
#[allow(dead_code)]
enum ColorMode {
    Hilbert,
    Velocity,
}

const MSAA_SAMPLE_COUNT: i32 = 8;
// Higher bits = finer Hilbert quantization (more distinct colors)
const HILBERT_BITS: u32 = 16;
const COLOR_MODE: ColorMode = ColorMode::Velocity;

fn rng_seed() -> u64 {
    env::var("BOIDS_SEED")
        .ok()
        .and_then(|s| s.parse::<u64>().ok())
        .unwrap_or(1)
}

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

    Color::new(r, g, b, 0.5)
}

fn rot(n: u32, x: &mut u32, y: &mut u32, rx: u32, ry: u32) {
    if ry == 0 {
        if rx == 1 {
            *x = n - 1 - *x;
            *y = n - 1 - *y;
        }
        std::mem::swap(x, y);
    }
}

fn hilbert_index(x: u32, y: u32, bits: u32) -> u64 {
    let mut d = 0u64;
    let mut x = x;
    let mut y = y;
    let n = 1u32 << bits;
    let mut s = n >> 1;
    while s > 0 {
        let rx = if (x & s) != 0 { 1 } else { 0 };
        let ry = if (y & s) != 0 { 1 } else { 0 };
        d += (s as u64) * (s as u64) * ((3 * rx) ^ ry) as u64;
        rot(n, &mut x, &mut y, rx, ry);
        s >>= 1;
    }
    d
}

fn hilbert_color(pos: Vec2, min: Vec2, max: Vec2, bits: u32) -> Color {
    let span = (max - min).max(Vec2::splat(1e-3));
    let max_val = ((1u32 << bits) - 1) as f32;
    let norm = Vec2::new(
        ((pos.x - min.x) / span.x).clamp(0.0, 1.0),
        ((pos.y - min.y) / span.y).clamp(0.0, 1.0),
    );
    let xi = (norm.x * max_val).round() as u32;
    let yi = (norm.y * max_val).round() as u32;
    let idx = hilbert_index(xi, yi, bits);
    let max_idx = (1u64 << (bits * 2)) - 1;
    let hue = idx as f32 / max_idx as f32;
    hsv_to_rgb(hue, 1.0, 1.0)
}

fn heading_color(forward: Vec2) -> Color {
    let angle = forward.y.atan2(forward.x);
    let hue = (angle / std::f32::consts::TAU).rem_euclid(1.0);
    hsv_to_rgb(hue, 1.0, 1.0)
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

fn draw_boid(boid: &Boid, scale: f32, min: Vec2, max: Vec2) {
    let pos = boid.pos;
    let vel = boid.vel;
    let speed2 = vel.length_squared();
    if speed2 < 1e-6 {
        return; // no direction → don't draw
    }

    let forward = vel.normalize();

    let color = match COLOR_MODE {
        ColorMode::Hilbert => hilbert_color(pos, min, max, HILBERT_BITS),
        ColorMode::Velocity => heading_color(forward),
    };

    let major = 8.0 * scale;
    let minor = 3.0 * scale;
    let angle = forward.y.atan2(forward.x).to_degrees();
    draw_ellipse(pos.x, pos.y, major, minor, angle, color);
}


#[macroquad::main(window_conf)]
async fn main() {
    // World matches your window size
    let bounds = WorldBounds {
        w: screen_width(),
        h: screen_height(),
    };

    let num_boids = 50_000;

    let seed = rng_seed();
    println!("Using RNG seed: {}", seed);
    let mut rng = StdRng::seed_from_u64(seed);

    // Use default boid behavior for now
    let behavior: BoidBehavior = BoidBehavior::default();

    // Create a random flock
    let flock = Flock::new_random_with_rng(num_boids, bounds, behavior, &mut rng);

    // let mut sim = Sim::new(flock, Box::new(SpatialHashNeighborSearch::new(behavior.neighbor_radius)), rng);
    // let mut sim = Sim::new(flock, Box::new(BruteForceNeighborSearch), rng);

    // let mut sim = Sim::new(flock, Box::new(QuadTreeNeighborSearch::new(8, 6)), rng);
    let mut sim = Sim::new(flock, Box::new(HilbertRotatedNeighborSearch::new(HILBERT_BITS)), rng);
    // let mut sim = Sim::new(flock, Box::new(HilbertNeighborSearch::new(HILBERT_BITS)), rng);

    let mut frame_times_ms: VecDeque<f32> = VecDeque::with_capacity(100);

    loop {
        let dt = get_frame_time();
        let start = Instant::now();
        sim.step(dt);
        let end = Instant::now();
        let duration = end.duration_since(start);
        // println!("Time taken: {:?}", duration);
        clear_background(BLACK);

        // Compute current extents to map Hilbert color to occupied space
        let (min, max) = sim
            .boids()
            .iter()
            .fold(None, |acc: Option<(Vec2, Vec2)>, b| match acc {
                None => Some((b.pos, b.pos)),
                Some((mn, mx)) => Some((
                    Vec2::new(mn.x.min(b.pos.x), mn.y.min(b.pos.y)),
                    Vec2::new(mx.x.max(b.pos.x), mx.y.max(b.pos.y)),
                )),
            })
            .unwrap_or((Vec2::ZERO, Vec2::new(bounds.w, bounds.h)));

        // Draw all boids
        for b in sim.boids() {
            draw_boid(b, 0.3, min, max);
        }

        let engine_ms = (duration.as_micros() as f32) / 1000.0;
        frame_times_ms.push_back(engine_ms);
        if frame_times_ms.len() > 100 {
            frame_times_ms.pop_front();
        }
        let avg_ms: f32 = if frame_times_ms.is_empty() {
            0.0
        } else {
            frame_times_ms.iter().copied().sum::<f32>() / frame_times_ms.len() as f32
        };
        println!("avg_ms: {}", avg_ms);
        // print()

        draw_text(
            format!(
                "Sim ({}) boids: {} avg_engine(100): {:.2}ms avg_fps: {:.2}",
                sim.algo_name(),
                num_boids,
                avg_ms,
                1000.0 / avg_ms
            )
            .as_str(),
            20.0,
            40.0,
            32.0,
            WHITE,
        );

        next_frame().await;
    }
}
