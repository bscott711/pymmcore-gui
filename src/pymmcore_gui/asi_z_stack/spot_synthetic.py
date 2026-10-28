"""Synthetic images of the CRISP LED patch, for tests and the fake camera.

Pure numpy. The model follows the one real screenshot we have (findings T02 B2):
a bright rounded square with a soft edge and an internal texture with dark
blotches, a faint tail up and right of the top-right corner, a dim ghost below
the patch (a wide, soft rectangle with a small brighter core), a flat dark
background, and sensor noise plus isolated hot pixels.

- :func:`render_led_patch` -- the noise-free float image.
- :func:`add_noise` -- sensor noise, hot pixels, rounding to an integer camera.

Only windows around the patch, the tail and the ghost are evaluated, so a
frame costs a few ms even on a 2048 x 1536 sensor.
"""

from __future__ import annotations

import math

import numpy as np

# Abramowitz & Stegun 7.1.26 (max error 1.5e-7): numpy has no erf and scipy is
# not a dependency.
_ERF_P = 0.3275911
_ERF_A = (0.254829592, -0.284496736, 1.421413741, -1.453152027, 1.061405429)
_SQRT2 = math.sqrt(2.0)

_CORNER = 0.12  # patch corner radius, fraction of the smaller side
_GHOST_SIZE = (0.90, 0.66)  # ghost size / patch size (277 x 188 vs 308 x 284 px)
_GHOST_BODY = 0.67  # ghost body brightness, in units of ``ghost * level``
_GHOST_CORE = 1.5  # extra brightness of the ghost's core, same units
_TAIL_CENTER = (0.60, -0.55)  # tail centre, in patch sizes from the patch centre
_TAIL_SIGMA = (0.15, 0.15)  # tail widths, in patch sizes
_N_WAVES = 4
_N_BLOTCHES = 6


def _soft_step(d: np.ndarray, sigma: float) -> np.ndarray:
    """Return ``0.5 * erfc(d / (sigma sqrt 2))``: 1 inside (d < 0), 0 outside.

    Exactly antisymmetric about 0.5 at ``d == 0``, so the 50 % point of an
    edge is where ``d == 0`` however large *sigma* is.
    """
    z = np.abs(d) * (1.0 / (max(sigma, 1e-6) * _SQRT2))
    t = 1.0 / (1.0 + _ERF_P * z)
    a1, a2, a3, a4, a5 = _ERF_A
    poly = t * (a1 + t * (a2 + t * (a3 + t * (a4 + t * a5))))
    tail = 0.5 * poly * np.exp(-z * z)
    out: np.ndarray = np.where(d >= 0, tail, 1.0 - tail)
    return out


def _rounded_box_distance(
    u: np.ndarray, v: np.ndarray, half_w: float, half_h: float, radius: float
) -> np.ndarray:
    """Signed distance to a rounded rectangle (negative inside)."""
    qx = np.abs(u) - (half_w - radius)
    qy = np.abs(v) - (half_h - radius)
    out: np.ndarray = (
        np.hypot(np.maximum(qx, 0.0), np.maximum(qy, 0.0))
        + np.minimum(np.maximum(qx, qy), 0.0)
        - radius
    )
    return out


def _window(
    shape: tuple[int, int], cx: float, cy: float, hx: float, hy: float
) -> tuple[int, int, int, int]:
    """Return ``(y0, y1, x0, x1)`` of the box ``c +- h`` clipped to the image."""
    rows, cols = shape
    x0, x1 = max(math.floor(cx - hx), 0), min(math.ceil(cx + hx) + 1, cols)
    y0, y1 = max(math.floor(cy - hy), 0), min(math.ceil(cy + hy) + 1, rows)
    return y0, y1, x0, x1


def _texture(
    u: np.ndarray,
    v: np.ndarray,
    size: tuple[float, float],
    strength: float,
    rng: np.random.Generator,
) -> np.ndarray:
    """Multiplicative texture in patch coordinates (moves with the patch).

    A few low-frequency waves of relative amplitude *strength* plus dark
    blotches up to ``2.5 * strength`` deep (capped at 90 %).
    """
    w, h = size
    field = np.zeros(np.broadcast(u, v).shape)
    for _ in range(_N_WAVES):
        lam = rng.uniform(0.4, 1.5) * min(w, h)
        phi = rng.uniform(0.0, 2.0 * math.pi)
        psi = rng.uniform(0.0, 2.0 * math.pi)
        k = 2.0 * math.pi / lam
        field += np.cos(k * (u * math.cos(phi) + v * math.sin(phi)) + psi)
    tex = 1.0 + strength * field / _N_WAVES * 2.0
    depth_max = min(0.9, 2.5 * strength)
    for _ in range(_N_BLOTCHES):
        ub = rng.uniform(-0.65, 0.65) * w / 2.0
        vb = rng.uniform(-0.65, 0.65) * h / 2.0
        rad = rng.uniform(0.08, 0.16) * min(w, h)
        depth = rng.uniform(0.5, 1.0) * depth_max
        tex = tex - depth * np.exp(-((u - ub) ** 2 + (v - vb) ** 2) / (2.0 * rad**2))
    out: np.ndarray = tex
    return out


def render_led_patch(
    shape: tuple[int, int],
    center: tuple[float, float],
    *,
    size: tuple[float, float] = (96, 92),
    angle_deg: float = 0.0,
    edge_sigma: float = 2.0,
    level: float = 100.0,
    background: float = 1.0,
    texture: float = 0.15,
    tail: float = 0.04,
    ghost: float = 0.3,
    ghost_offset: tuple[float, float] = (-0.11, 1.35),
    ghost_center: tuple[float, float] | None = None,
    seed: int = 0,
) -> np.ndarray:
    """Render the LED patch as a noise-free float64 image.

    Pixel centres are at integer coordinates, so an edge at ``x = e`` has its
    50 % point exactly at ``e`` whatever *edge_sigma* is (symmetric blur).

    Parameters
    ----------
    shape : tuple[int, int]
        Image ``(rows, cols)``.
    center : tuple[float, float]
        Patch centre ``(x, y)`` in pixels.
    size : tuple[float, float]
        Patch ``(width, height)`` in pixels (the edges are at ``center +-
        size / 2`` before rotation).
    angle_deg : float
        Rotation of the patch. A vertical edge then has an image slope
        ``dx/dy = tan(angle_deg)``, a horizontal edge ``dy/dx = -tan(angle_deg)``.
    edge_sigma : float
        Gaussian blur of the edge in pixels (the 25 -> 50 % rise is about
        ``0.67 * edge_sigma``).
    level : float
        Patch brightness above *background*, before texture.
    background : float
        Flat pedestal added everywhere.
    texture : float
        Relative strength of the internal texture and dark blotches (0 = flat).
    tail : float
        Peak of the faint tail up and right of the top-right corner, as a
        fraction of *level* (0 = none).
    ghost : float
        Strength of the dim ghost below the patch (0 = none). The body is
        ``0.67 * ghost * level``, the core adds ``1.5 * ghost * level``.
    ghost_offset : tuple[float, float]
        Ghost centre relative to the patch centre, in patch widths / heights.
    ghost_center : tuple[float, float] | None
        Explicit ghost centre ``(x, y)`` in pixels; overrides *ghost_offset*.
    seed : int
        Seed of the texture (the same seed gives the same texture).

    Returns
    -------
    np.ndarray
        float64 image of *shape*.
    """
    rows, cols = int(shape[0]), int(shape[1])
    img = np.full((rows, cols), float(background), dtype=np.float64)
    cx, cy = float(center[0]), float(center[1])
    w, h = float(size[0]), float(size[1])
    sigma = float(edge_sigma)

    # -- patch -------------------------------------------------------------
    th = math.radians(angle_deg)
    c, s = math.cos(th), math.sin(th)
    margin = 6.0 * sigma + 2.0
    hx = w / 2.0 * abs(c) + h / 2.0 * abs(s) + margin
    hy = w / 2.0 * abs(s) + h / 2.0 * abs(c) + margin
    y0, y1, x0, x1 = _window((rows, cols), cx, cy, hx, hy)
    if y1 > y0 and x1 > x0 and level != 0:
        dx = np.arange(x0, x1, dtype=np.float64)[None, :] - cx
        dy = np.arange(y0, y1, dtype=np.float64)[:, None] - cy
        # image -> patch frame (the inverse of the rotation described above)
        u = dx * c - dy * s
        v = dx * s + dy * c
        dist = _rounded_box_distance(u, v, w / 2.0, h / 2.0, _CORNER * min(w, h))
        patch = level * _soft_step(dist, sigma)
        if texture > 0:
            patch = patch * _texture(u, v, (w, h), texture, np.random.default_rng(seed))
        img[y0:y1, x0:x1] += patch

    # -- tail --------------------------------------------------------------
    if tail > 0 and level != 0:
        tx, ty = cx + _TAIL_CENTER[0] * w, cy + _TAIL_CENTER[1] * h
        sx, sy = _TAIL_SIGMA[0] * w, _TAIL_SIGMA[1] * h
        y0, y1, x0, x1 = _window((rows, cols), tx, ty, 5 * sx, 5 * sy)
        if y1 > y0 and x1 > x0:
            gx = np.exp(-((np.arange(x0, x1) - tx) ** 2) / (2 * sx**2))
            gy = np.exp(-((np.arange(y0, y1) - ty) ** 2) / (2 * sy**2))
            img[y0:y1, x0:x1] += tail * level * np.outer(gy, gx)

    # -- ghost -------------------------------------------------------------
    if ghost > 0 and level != 0:
        if ghost_center is None:
            gcx, gcy = cx + ghost_offset[0] * w, cy + ghost_offset[1] * h
        else:
            gcx, gcy = float(ghost_center[0]), float(ghost_center[1])
        gw, gh = _GHOST_SIZE[0] * w, _GHOST_SIZE[1] * h
        gsig = 3.0 * sigma  # the ghost is softer than the patch
        core_sig = 0.12 * w
        y0, y1, x0, x1 = _window(
            (rows, cols),
            gcx,
            gcy,
            gw / 2.0 + 6 * gsig + 2,
            gh / 2.0 + 6 * gsig + 2,
        )
        if y1 > y0 and x1 > x0:
            gdx = np.arange(x0, x1, dtype=np.float64)[None, :] - gcx
            gdy = np.arange(y0, y1, dtype=np.float64)[:, None] - gcy
            gdist = _rounded_box_distance(
                gdx, gdy, gw / 2.0, gh / 2.0, 0.2 * min(gw, gh)
            )
            body = _GHOST_BODY * ghost * level * _soft_step(gdist, gsig)
            core = (
                _GHOST_CORE
                * ghost
                * level
                * np.exp(-(gdx**2 + gdy**2) / (2.0 * core_sig**2))
            )
            img[y0:y1, x0:x1] += body + core
    return img


def add_noise(
    img: np.ndarray,
    *,
    noise: float,
    rng: np.random.Generator,
    hot_pixels: int = 0,
    bits: int = 8,
) -> np.ndarray:
    """Add sensor noise and hot pixels, round and clip to an integer camera.

    Parameters
    ----------
    img : np.ndarray
        Float image in counts.
    noise : float
        Standard deviation of the Gaussian noise in counts (0 = none).
    rng : np.random.Generator
        Source of randomness (also used for the hot pixel positions).
    hot_pixels : int
        Number of isolated hot pixels, each set to 60-90 % of full scale.
    bits : int
        Bit depth: ``uint8`` for ``bits <= 8``, else ``uint16``.

    Returns
    -------
    np.ndarray
        ``uint8`` or ``uint16`` image of the same shape.
    """
    top = float((1 << int(bits)) - 1)
    # float32 holds every 16-bit count exactly and halves the memory traffic.
    if noise > 0:
        out = rng.standard_normal(img.shape, dtype=np.float32)
        out *= np.float32(noise)
        out += img
    else:
        out = np.array(img, dtype=np.float32)
    if hot_pixels > 0:
        rr = rng.integers(0, out.shape[0], int(hot_pixels))
        cc = rng.integers(0, out.shape[1], int(hot_pixels))
        out[rr, cc] = rng.uniform(0.6, 0.9, int(hot_pixels)) * top
    np.rint(out, out=out)
    np.clip(out, 0.0, top, out=out)
    result: np.ndarray = out.astype(np.uint8 if bits <= 8 else np.uint16)
    return result


__all__ = ["add_noise", "render_led_patch"]
