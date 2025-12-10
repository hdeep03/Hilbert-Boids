use std::cmp::Ordering;

use macroquad::prelude::Vec2;
use rand::Rng;

use super::NeighborSearch;
use crate::boid::{Boid, BoidBehavior};

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

#[allow(dead_code)]
pub struct HilbertRotatedNeighborSearch {
    bits: u32,
    items_xy: Vec<(u64, usize, Vec2)>,
    items_yx: Vec<(u64, usize, Vec2)>,
    min: Vec2,
    max: Vec2,
    rot_cos: f32,
    rot_sin: f32,
}

#[allow(dead_code)]
impl HilbertRotatedNeighborSearch {
    #[allow(dead_code)]
    pub fn new(bits: u32) -> Self {
        Self {
            bits: bits.max(2),
            items_xy: Vec::new(),
            items_yx: Vec::new(),
            min: Vec2::ZERO,
            max: Vec2::ZERO,
            rot_cos: 1.0,
            rot_sin: 0.0,
        }
    }

    fn rotate(&self, p: Vec2) -> Vec2 {
        Vec2::new(
            self.rot_cos * p.x - self.rot_sin * p.y,
            self.rot_sin * p.x + self.rot_cos * p.y,
        )
    }

    fn normalize_pos(&self, p: Vec2) -> (u32, u32) {
        let p = self.rotate(p);
        let span = (self.max - self.min).max(Vec2::splat(1e-3));
        let max_val = ((1u32 << self.bits) - 1) as f32;
        let norm = (p - self.min) / span;
        let clamped = Vec2::new(norm.x.clamp(0.0, 1.0), norm.y.clamp(0.0, 1.0));
        let xi = (clamped.x * max_val).round() as u32;
        let yi = (clamped.y * max_val).round() as u32;
        (xi, yi)
    }

    #[allow(unused_variables)]
    fn collect_from(
        items: &[(u64, usize, Vec2)],
        query_key: u64,
        boids: &[Boid],
        behavior: &BoidBehavior,
        index: usize,
        acc: &mut Vec<(f32, usize)>,
        limit: usize,
    ) {
        if items.is_empty() || limit == 0 {
            return;
        }

        let pos = boids[index].pos;
        let r2 = behavior.neighbor_radius * behavior.neighbor_radius;

        let pos_idx = match items.binary_search_by(|item| item.0.cmp(&query_key)) {
            Ok(p) => p,
            Err(p) => p,
        };

        let mut left = pos_idx as isize - 1;
        let mut right = pos_idx as isize;

        while (left >= 0 || right < items.len() as isize) && acc.len() < limit {
            let step_left = left >= 0;
            let step_right = right < items.len() as isize;

            let choose_left = if step_left && step_right {
                let k_left = items[left as usize].0;
                let k_right = items[right as usize].0;
                (query_key as i128 - k_left as i128).abs()
                    <= (k_right as i128 - query_key as i128).abs()
            } else {
                step_left
            };

            let (_, j, p) = if choose_left {
                let tup = items[left as usize];
                left -= 1;
                tup
            } else {
                let tup = items[right as usize];
                right += 1;
                tup
            };

            if j == index {
                continue;
            }

            let dist2 = (p - pos).length_squared();
            if dist2 > r2 || dist2 <= 1e-6 {
                continue;
            }

            if acc.iter().any(|&(_, idx)| idx == j) {
                continue;
            }

            acc.push((dist2, j));
        }
    }
}

impl NeighborSearch for HilbertRotatedNeighborSearch {
    fn rebuild(&mut self, boids: &[Boid]) {
        self.items_xy.clear();
        self.items_yx.clear();

        if boids.is_empty() {
            return;
        }

        // Pick a fresh random rotation each rebuild.
        let mut rng = rand::rng();
        let angle = rng.random_range(0.0..std::f32::consts::TAU);
        self.rot_cos = angle.cos();
        self.rot_sin = angle.sin();

        let mut min = boids[0].pos;
        let mut max = boids[0].pos;
        for b in boids.iter().skip(1) {
            let rp = self.rotate(b.pos);
            min.x = min.x.min(rp.x);
            min.y = min.y.min(rp.y);
            max.x = max.x.max(rp.x);
            max.y = max.y.max(rp.y);
        }

        // Add a tiny padding to avoid zero span.
        let padding = 1e-3;
        min -= Vec2::splat(padding);
        max += Vec2::splat(padding);
        self.min = min;
        self.max = max;

        self.items_xy.reserve(boids.len());
        self.items_yx.reserve(boids.len());

        for (idx, b) in boids.iter().enumerate() {
            let (xi, yi) = self.normalize_pos(b.pos);
            let key_xy = hilbert_index(xi, yi, self.bits);
            let key_yx = hilbert_index(yi, xi, self.bits);
            self.items_xy.push((key_xy, idx, b.pos));
            self.items_yx.push((key_yx, idx, b.pos));
        }

        self.items_xy
            .sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(Ordering::Equal));
        self.items_yx
            .sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(Ordering::Equal));
    }

    fn neighbors(&self, boids: &[Boid], behavior: &BoidBehavior, index: usize) -> Vec<usize> {
        if boids.is_empty() {
            return Vec::new();
        }

        let limit = behavior.neighbor_limit;
        if limit == 0 {
            return Vec::new();
        }

        let (xi, yi) = self.normalize_pos(boids[index].pos);
        let key_xy = hilbert_index(xi, yi, self.bits);
        let key_yx = hilbert_index(yi, xi, self.bits);

        let mut acc: Vec<(f32, usize)> = Vec::with_capacity(limit * 2);
        Self::collect_from(
            &self.items_xy,
            key_xy,
            boids,
            behavior,
            index,
            &mut acc,
            limit,
        );
        Self::collect_from(
            &self.items_yx,
            key_yx,
            boids,
            behavior,
            index,
            &mut acc,
            limit,
        );

        // acc.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(Ordering::Equal));
        acc.into_iter().take(limit).map(|(_, idx)| idx).collect()
    }

    fn name(&self) -> &'static str {
        "HilbertRotated"
    }
}
