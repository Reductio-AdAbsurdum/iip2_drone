#!/usr/bin/env python3
"""
Mission Planner indoor custom map server.

Generates a small floor-plan-style tile pyramid centered on your lab's GPS
coordinates, and serves it on http://localhost:8080/{z}/{x}/{y}.png so
Mission Planner can use it as a Custom Map source. The tiles are rendered
on demand from your anchor layout -- no pre-generation needed.

Setup:
    pip install pillow

Run:
    python3 mp_indoor_map.py

Mission Planner config (one-time):
    1. Make sure this script is running (you'll see "Tile server running...").
    2. Mission Planner -> Flight Data -> right-click on the map.
    3. Map Type -> "Custom WMS" or "Custom" (label varies by MP version).
    4. Paste:    http://localhost:8080/{z}/{x}/{y}.png
    5. Pan/zoom to your lab's lat/lon (set in CONFIG below). The synthetic
       floor plan with your anchors should appear.

If the map doesn't update after you change ANCHOR_XY:
    Mission Planner caches tiles aggressively. Either restart MP, or clear
    its tile cache (typically ~/Library/MissionPlanner/gmapcache/ on Mac,
    %LOCALAPPDATA%/MissionPlanner/gmapcache/ on Windows).
"""
import math
import io
import sys
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

try:
    from PIL import Image, ImageDraw, ImageFont
except ImportError:
    print("Missing dependency. Install with:  pip install pillow")
    sys.exit(1)


# ============================================================================
# CONFIG -- update these to match your lab and anchor layout.
# ============================================================================

# Your actual lab GPS. Look up your building in Google Maps, right-click the
# spot, copy the lat/lon. This is what the EKF origin in temp.ino should also
# match for the drone icon to land on the floor plan in MP.
LAB_LAT = 47.171850
LAB_LON = 8.464150

# Anchor positions in NED (meters) -- same as ANCHOR_XY in temp.ino / uwb_plot.py.
# (X = North, Y = East)
ANCHOR_XY = [
    (0.00, 0.00),   # A0
    (4.00, 0.00),   # A1
    (4.00, 4.00),   # A2
    (0.00, 4.00),   # A3
]

# How big a square (meters) to draw around the origin. Use enough to cover
# anchors plus a little margin. 8-10 m is fine for a 4 m room.
ROOM_SIZE_M = 8.0

# Server settings.
PORT = 8080
TILE_SIZE = 256

# ============================================================================


# Earth flattening helpers. Tiny indoor distances, so a flat-earth approximation
# (constant meters-per-degree at this latitude) is plenty accurate.
M_PER_DEG_LAT = 111_111.0
M_PER_DEG_LON = 111_111.0 * math.cos(math.radians(LAB_LAT))


def tile_bounds(z: int, x: int, y: int):
    """Returns the lat/lon bounds of an XYZ tile (top-left and bottom-right corners)."""
    n = 2 ** z
    lon_left = x / n * 360.0 - 180.0
    lon_right = (x + 1) / n * 360.0 - 180.0
    lat_top = math.degrees(math.atan(math.sinh(math.pi * (1 - 2 * y / n))))
    lat_bottom = math.degrees(math.atan(math.sinh(math.pi * (1 - 2 * (y + 1) / n))))
    return lat_top, lon_left, lat_bottom, lon_right


def latlon_to_local_m(lat: float, lon: float):
    """Lat/lon -> local meters from (LAB_LAT, LAB_LON). Returns (north, east)."""
    n = (lat - LAB_LAT) * M_PER_DEG_LAT
    e = (lon - LAB_LON) * M_PER_DEG_LON
    return n, e


def render_tile(z: int, x: int, y: int) -> Image.Image:
    """Render one 256x256 tile covering the given XYZ slot."""
    lat_top, lon_left, lat_bottom, lon_right = tile_bounds(z, x, y)
    n_top, e_left = latlon_to_local_m(lat_top, lon_left)
    n_bottom, e_right = latlon_to_local_m(lat_bottom, lon_right)

    width_m = e_right - e_left
    height_m = n_top - n_bottom

    half = ROOM_SIZE_M / 2.0 + 1.0  # small extra margin

    # If tile doesn't intersect the lab area, return transparent.
    if (e_right < -half or e_left > half or
            n_top < -half or n_bottom > half):
        return Image.new("RGBA", (TILE_SIZE, TILE_SIZE), (0, 0, 0, 0))

    # Render floor plan onto this tile.
    img = Image.new("RGBA", (TILE_SIZE, TILE_SIZE), (245, 245, 240, 255))
    d = ImageDraw.Draw(img)

    # Helper: meters -> pixel within this tile.
    def m_to_px(north: float, east: float):
        x_px = (east - e_left) / width_m * TILE_SIZE
        y_px = (n_top - north) / height_m * TILE_SIZE
        return x_px, y_px

    # 1 m grid.
    grid_min = -int(half) - 1
    grid_max = int(half) + 2
    for m in range(grid_min, grid_max):
        x_px, _ = m_to_px(0, m)
        if 0 <= x_px <= TILE_SIZE:
            color = (140, 140, 140) if m == 0 else (200, 200, 200)
            width = 2 if m == 0 else 1
            d.line([(x_px, 0), (x_px, TILE_SIZE)], fill=color, width=width)
        _, y_px = m_to_px(m, 0)
        if 0 <= y_px <= TILE_SIZE:
            color = (140, 140, 140) if m == 0 else (200, 200, 200)
            width = 2 if m == 0 else 1
            d.line([(0, y_px), (TILE_SIZE, y_px)], fill=color, width=width)

    # Anchor markers.
    for i, (n_m, e_m) in enumerate(ANCHOR_XY):
        cx, cy = m_to_px(n_m, e_m)
        if -20 <= cx <= TILE_SIZE + 20 and -20 <= cy <= TILE_SIZE + 20:
            r = 8
            d.ellipse([cx - r, cy - r, cx + r, cy + r],
                      fill=(70, 130, 180, 255), outline=(0, 0, 0, 255), width=2)
            d.text((cx + r + 2, cy - r - 2), f"A{i}", fill=(0, 0, 0, 255))

    # Origin marker (small green cross).
    ox, oy = m_to_px(0, 0)
    if 0 <= ox <= TILE_SIZE and 0 <= oy <= TILE_SIZE:
        d.line([(ox - 6, oy), (ox + 6, oy)], fill=(0, 130, 0), width=2)
        d.line([(ox, oy - 6), (ox, oy + 6)], fill=(0, 130, 0), width=2)

    # North arrow + label in top-left corner if this tile contains origin.
    if -2 < n_top and 2 > n_bottom and -2 < e_left and 2 > e_right:
        d.text((4, 4), "N ↑", fill=(0, 0, 0))

    return img


class TileHandler(BaseHTTPRequestHandler):
    def do_GET(self):
        # Path forms we accept: /{z}/{x}/{y}.png  or  /{z}/{x}/{y}
        parts = self.path.strip("/").split("/")
        if len(parts) != 3:
            self.send_response(404)
            self.end_headers()
            return
        try:
            z = int(parts[0])
            x = int(parts[1])
            y = int(parts[2].split(".")[0])
        except ValueError:
            self.send_response(400)
            self.end_headers()
            return

        try:
            img = render_tile(z, x, y)
        except Exception as e:
            self.send_response(500)
            self.end_headers()
            self.wfile.write(f"render error: {e}".encode())
            return

        buf = io.BytesIO()
        img.save(buf, format="PNG")
        body = buf.getvalue()

        self.send_response(200)
        self.send_header("Content-Type", "image/png")
        self.send_header("Content-Length", str(len(body)))
        self.send_header("Cache-Control", "no-store")
        self.send_header("Access-Control-Allow-Origin", "*")
        self.end_headers()
        self.wfile.write(body)

    def log_message(self, fmt, *args):
        # Quiet the per-request stderr spam; uncomment for debugging.
        # super().log_message(fmt, *args)
        return


def main():
    server = ThreadingHTTPServer(("0.0.0.0", PORT), TileHandler)
    print(f"Tile server running on http://localhost:{PORT}/{{z}}/{{x}}/{{y}}.png")
    print(f"Lab origin: ({LAB_LAT}, {LAB_LON})")
    print(f"Anchors: {ANCHOR_XY}")
    print("Mission Planner: right-click map -> Map Type -> Custom WMS / Custom URL.")
    print("Paste the URL above. Pan to the lab origin and zoom in (z=20+).")
    print("Ctrl+C to stop.")
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\nStopped.")


if __name__ == "__main__":
    main()
