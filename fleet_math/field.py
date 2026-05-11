"""Continuous constraint field.

Embeds weighted positions in 2D and provides field interpolation via
inverse-distance weighting, gradient computation, and gap detection.
"""

import math


class Field:
    """A 2D scalar field defined at sampled positions with weights.

    The field value at any query point is computed as the inverse-distance
    weighted average of nearby embedded positions, using a power parameter
    p (default 2.0 for a natural falloff).
    """

    def __init__(self, power=2.0):
        self._points = {}  # {position_id: (x, y, weight)}
        self._power = power

    @property
    def points(self):
        return dict(self._points)

    @property
    def positions(self):
        """Return dict {position_id: (x, y, weight)} for compatibility."""
        return dict(self._points)

    def embed(self, position_id, x, y, weight=1.0):
        """Embed a weighted position in the field.

        If position_id already exists, updates its position and weight.
        """
        self._points[position_id] = (float(x), float(y), float(weight))

    def remove(self, position_id):
        """Remove an embedded position from the field."""
        self._points.pop(position_id, None)

    def read(self, query_x, query_y, min_neighbors=1):
        """Evaluate field value at (query_x, query_y).

        Uses inverse-distance weighted interpolation. Returns 0.0 if
        fewer than min_neighbors points are within computational range.

        Args:
            query_x: X-coordinate of query point.
            query_y: Y-coordinate of query point.
            min_neighbors: Minimum number of points required to interpolate.

        Returns:
            Interpolated field value (float), or 0.0 if insufficient data.
        """
        if not self._points:
            return 0.0

        total_weight = 0.0
        weighted_sum = 0.0
        points_within_range = 0

        for _pid, (px, py, w) in self._points.items():
            dx = query_x - px
            dy = query_y - py
            dist = math.hypot(dx, dy)

            if dist == 0.0:
                # Exact match — return that point's field value directly
                return w

            # Inverse distance with power p
            inv_dist = 1.0 / (dist ** self._power)
            weighted_sum += w * inv_dist
            total_weight += inv_dist
            points_within_range += 1

        if points_within_range < min_neighbors or total_weight == 0.0:
            return 0.0

        return weighted_sum / total_weight

    def gradients(self, resolution=20):
        """Compute field gradients on a grid.

        Returns list of dicts: [{x, y, gx, gy, magnitude}]
        """
        if not self._points:
            return []

        xs = [p[0] for p in self._points.values()]
        ys = [p[1] for p in self._points.values()]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)

        pad_x = (max_x - min_x) * 0.1 if max_x > min_x else 1.0
        pad_y = (max_y - min_y) * 0.1 if max_y > min_y else 1.0

        step_x = (max_x - min_x + 2 * pad_x) / max(resolution, 2)
        step_y = (max_y - min_y + 2 * pad_y) / max(resolution, 2)

        results = []
        eps = step_x * 0.01  # Small step for numerical gradient

        i = 0
        x = min_x - pad_x
        while x <= max_x + pad_x and i < resolution:
            j = 0
            y = min_y - pad_y
            while y <= max_y + pad_y and j < resolution:
                f_center = self.read(x, y)
                f_dx = self.read(x + eps, y)
                f_dy = self.read(x, y + eps)

                gx = (f_dx - f_center) / eps if eps != 0 else 0.0
                gy = (f_dy - f_center) / eps if eps != 0 else 0.0
                mag = math.hypot(gx, gy)

                results.append({
                    'x': x,
                    'y': y,
                    'gx': gx,
                    'gy': gy,
                    'magnitude': mag,
                })
                j += 1
                y += step_y
            i += 1
            x += step_x

        return results

    def gaps(self, grid_size=10, density_threshold=0.5):
        """Find regions with low field density (gaps).

        Divides the field bounding box into a grid, counts the number
        of embedded points per cell, and returns cells whose density
        falls below density_threshold * max_density.

        Args:
            grid_size: Number of cells along each axis.
            density_threshold: Fraction of max cell density (0-1).

        Returns:
            List of {x: center_x, y: center_y, count: point_count} for
            cells below the density threshold.
        """
        if not self._points:
            return []

        xs = [p[0] for p in self._points.values()]
        ys = [p[1] for p in self._points.values()]
        min_x, max_x = min(xs), max(xs)
        min_y, max_y = min(ys), max(ys)

        pad_x = (max_x - min_x) * 0.05 if max_x > min_x else 1.0
        pad_y = (max_y - min_y) * 0.05 if max_y > min_y else 1.0

        x0, x1 = min_x - pad_x, max_x + pad_x
        y0, y1 = min_y - pad_y, max_y + pad_y

        if x1 <= x0 or y1 <= y0:
            return []

        cell_w = (x1 - x0) / grid_size
        cell_h = (y1 - y0) / grid_size

        # Count points per cell
        counts = [[0] * grid_size for _ in range(grid_size)]
        for _pid, (px, py, _w) in self._points.items():
            ci = min(int((px - x0) / cell_w), grid_size - 1)
            cj = min(int((py - y0) / cell_h), grid_size - 1)
            ci = max(0, ci)
            cj = max(0, cj)
            counts[ci][cj] += 1

        # Find max density
        max_count = max(max(row) for row in counts)
        if max_count == 0:
            return []

        threshold = max_count * density_threshold

        gaps = []
        for i in range(grid_size):
            for j in range(grid_size):
                if counts[i][j] < threshold:
                    gaps.append({
                        'x': x0 + (i + 0.5) * cell_w,
                        'y': y0 + (j + 0.5) * cell_h,
                        'count': counts[i][j],
                    })

        return gaps
