"""Rolling 2D occupancy grid (TP-22), inflation (TP-23) and decay (TP-24)."""
import math

UNKNOWN, FREE, OCCUPIED = -1, 0, 100   # nav_msgs/OccupancyGrid values


class RollingGrid:

    def __init__(self, resolution, width, height):
        """resolution in m/cell; width and height in m. Centred on the drone."""
        if not all(math.isfinite(v) and v > 0 for v in (resolution, width, height)):
            raise ValueError('resolution, width and height must be positive and finite')
        self.resolution = float(resolution)
        self.width = float(width)
        self.height = float(height)
        self.columns = max(1, int(round(width/resolution)))
        self.rows = max(1, int(round(height/resolution)))
        # Lower-left corner of the window in the map frame, aligned to the cell size.
        self.origin = (-(self.columns//2)*self.resolution, -(self.rows//2)*self.resolution)
        self.cells = [UNKNOWN]*(self.columns*self.rows)
        self.seen = [-math.inf]*(self.columns*self.rows)

    def recenter(self, x, y):
        """Shift the window so the drone stays at its centre, keeping observed cells."""
        ox = (math.floor(x/self.resolution) - self.columns//2)*self.resolution
        oy = (math.floor(y/self.resolution) - self.rows//2)*self.resolution
        shift_x = int(round((ox-self.origin[0])/self.resolution))
        shift_y = int(round((oy-self.origin[1])/self.resolution))
        if shift_x == 0 and shift_y == 0:
            return
        cells, seen = [UNKNOWN]*len(self.cells), [-math.inf]*len(self.seen)
        for j in range(self.rows):
            old_j = j + shift_y
            if not 0 <= old_j < self.rows:
                continue
            for i in range(self.columns):
                old_i = i + shift_x
                if 0 <= old_i < self.columns:
                    cells[j*self.columns+i] = self.cells[old_j*self.columns+old_i]
                    seen[j*self.columns+i] = self.seen[old_j*self.columns+old_i]
        self.cells, self.seen = cells, seen
        self.origin = (ox, oy)

    def world_to_cell(self, x, y):
        """(i, j) of the cell containing (x, y), or None outside the window."""
        i = math.floor((x-self.origin[0])/self.resolution)
        j = math.floor((y-self.origin[1])/self.resolution)
        if 0 <= i < self.columns and 0 <= j < self.rows:
            return i, j
        return None

    def cell_to_world(self, i, j):
        """Centre of cell (i, j) in the map frame."""
        return (self.origin[0] + (i+.5)*self.resolution,
                self.origin[1] + (j+.5)*self.resolution)

    def value(self, x, y):
        cell = self.world_to_cell(x, y)
        return None if cell is None else self.cells[cell[1]*self.columns+cell[0]]

    def integrate_ray(self, t, origin, end, hit):
        """Mark the ray cells free (Bresenham) and the final cell occupied.

        Only a ray that got an echo back carries evidence: it proves both that
        the end point is solid and that the space before it is clear, so it also
        clears an obstacle that has been removed. A ray without return (hit is
        False) may be a dark or glass surface and is ignored, because treating it
        as free space would erase obstacles the map has already seen.
        """
        if not hit:
            return
        start = self._unclamped_cell(origin[0], origin[1])
        stop = self._unclamped_cell(end[0], end[1])
        for i, j in list(_bresenham(start, stop))[:-1]:
            self._set(i, j, FREE, t)
        self._set(*stop, OCCUPIED, t)

    def decay(self, now, decay_time):
        """Return to UNKNOWN the occupied cells not re-observed within decay_time s."""
        for index, value in enumerate(self.cells):
            if value == OCCUPIED and now - self.seen[index] > decay_time:
                self.cells[index] = UNKNOWN

    def inflated(self, radius):
        """Copy of the grid with obstacles inflated by `radius` m (drone radius + margin)."""
        if not math.isfinite(radius) or radius < 0:
            raise ValueError('radius must be non-negative and finite')
        reach = int(math.ceil(radius/self.resolution))
        offsets = [(di, dj) for dj in range(-reach, reach+1) for di in range(-reach, reach+1)
                   if math.hypot(di, dj)*self.resolution <= radius]
        out = list(self.cells)
        for index, value in enumerate(self.cells):
            if value != OCCUPIED:
                continue
            i, j = index % self.columns, index // self.columns
            for di, dj in offsets:
                ni, nj = i+di, j+dj
                if 0 <= ni < self.columns and 0 <= nj < self.rows:
                    out[nj*self.columns+ni] = OCCUPIED
        return out

    def _unclamped_cell(self, x, y):
        return (math.floor((x-self.origin[0])/self.resolution),
                math.floor((y-self.origin[1])/self.resolution))

    def _set(self, i, j, value, t):
        if 0 <= i < self.columns and 0 <= j < self.rows:
            index = j*self.columns+i
            self.cells[index] = value
            self.seen[index] = t


def _bresenham(start, stop):
    """Integer cells from start to stop, both included."""
    (i, j), (i1, j1) = start, stop
    di, dj = abs(i1-i), -abs(j1-j)
    si, sj = (1 if i < i1 else -1), (1 if j < j1 else -1)
    error = di + dj
    while True:
        yield i, j
        if (i, j) == (i1, j1):
            return
        doubled = 2*error
        if doubled >= dj:
            error += dj
            i += si
        if doubled <= di:
            error += di
            j += sj
