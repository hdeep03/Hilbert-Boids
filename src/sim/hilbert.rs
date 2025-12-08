use std::cmp::Ordering;

use macroquad::prelude::Vec2;

use crate::boid::{Boid, BoidBehavior};
use super::NeighborSearch;

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
    let n = 1u32 << bits;            // side length
    let mut s = n >> 1;              // highest bit
    while s > 0 {
        let rx = if (x & s) != 0 { 1 } else { 0 };
        let ry = if (y & s) != 0 { 1 } else { 0 };
        d += (s as u64) * (s as u64) * ((3 * rx) ^ ry) as u64;
        rot(n, &mut x, &mut y, rx, ry);
        s >>= 1;
    }
    d
}

pub struct HilbertNeighborSearch {
    bits: u32,
    items: Vec<(u64, usize, Vec2)>, // (hilbert_key, index, pos)
    min: Vec2,
    max: Vec2,
}

impl HilbertNeighborSearch {
    #[allow(dead_code)]
    /// `bits` controls quantization; 16 bits gives a 65536x65536 grid.
    pub fn new(bits: u32) -> Self {
        Self {
            bits: bits.max(2),
            items: Vec::new(),
            min: Vec2::ZERO,
            max: Vec2::ZERO,
        }
    }

    fn normalize_pos(&self, p: Vec2) -> (u32, u32) {
        let span = (self.max - self.min).max(Vec2::splat(1e-3));
        let max_val = ((1u32 << self.bits) - 1) as f32;
        let norm = (p - self.min) / span;
        let clamped = Vec2::new(norm.x.clamp(0.0, 1.0), norm.y.clamp(0.0, 1.0));
        let xi = (clamped.x * max_val).round() as u32;
        let yi = (clamped.y * max_val).round() as u32;
        (xi, yi)
    }
}

impl NeighborSearch for HilbertNeighborSearch {
    fn rebuild(&mut self, boids: &[Boid]) {
        self.items.clear();

        if boids.is_empty() {
            return;
        }

        let mut min = boids[0].pos;
        let mut max = boids[0].pos;
        for b in boids.iter().skip(1) {
            min.x = min.x.min(b.pos.x);
            min.y = min.y.min(b.pos.y);
            max.x = max.x.max(b.pos.x);
            max.y = max.y.max(b.pos.y);
        }

        // Add a tiny padding to avoid zero span.
        let padding = 1e-3;
        min -= Vec2::splat(padding);
        max += Vec2::splat(padding);
        self.min = min;
        self.max = max;

        self.items.reserve(boids.len());
        for (idx, b) in boids.iter().enumerate() {
            let (xi, yi) = self.normalize_pos(b.pos);
            let key = hilbert_index(xi, yi, self.bits);
            self.items.push((key, idx, b.pos));
        }

        self.items
            .sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(Ordering::Equal));
    }

    fn neighbors(&self, boids: &[Boid], behavior: &BoidBehavior, index: usize) -> Vec<usize> {
        if self.items.is_empty() {
            return Vec::new();
        }

        let limit = behavior.neighbor_limit;
        if limit == 0 {
            return Vec::new();
        }

        let (xi, yi) = self.normalize_pos(boids[index].pos);
        let query_key = hilbert_index(xi, yi, self.bits);

        // Locate insertion point.
        let pos = match self.items.binary_search_by(|(k, _, _)| k.cmp(&query_key)) {
            Ok(p) => p,
            Err(p) => p,
        };

        let r2 = behavior.neighbor_radius * behavior.neighbor_radius;
        let mut acc: Vec<(f32, usize)> = Vec::with_capacity(limit);

        let mut left = pos as isize - 1;
        let mut right = pos as isize;

        while (left >= 0 || right < self.items.len() as isize) && acc.len() < limit {
            // Decide which direction to step: pick the closer key if both sides available.
            let step_left = left >= 0;
            let step_right = right < self.items.len() as isize;

            let choose_left = if step_left && step_right {
                let k_left = self.items[left as usize].0;
                let k_right = self.items[right as usize].0;
                (query_key as i128 - k_left as i128).abs() <= (k_right as i128 - query_key as i128).abs()
            } else {
                step_left
            };

            if choose_left {
                let (_, j, p) = self.items[left as usize];
                if j != index {
                    let dist2 = (p - boids[index].pos).length_squared();
                    if dist2 <= r2 && dist2 > 1e-6 {
                        acc.push((dist2, j));
                    }
                }
                left -= 1;
            } else if step_right {
                let (_, j, p) = self.items[right as usize];
                if j != index {
                    let dist2 = (p - boids[index].pos).length_squared();
                    if dist2 <= r2 && dist2 > 1e-6 {
                        acc.push((dist2, j));
                    }
                }
                right += 1;
            } else {
                break;
            }
        }

        acc.sort_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(Ordering::Equal));
        acc.into_iter().take(limit).map(|(_, idx)| idx).collect()
    }

    fn name(&self) -> &'static str {
        "Hilbert"
    }
}

