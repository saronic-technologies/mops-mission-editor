# route_adapter_with_controls.py
import json
import math
import sys
import difflib
from typing import Any, Dict, List, Tuple, Optional

from PySide6.QtCore import Qt, QRectF, QPointF
from PySide6.QtGui import QAction, QPen, QBrush, QColor
from PySide6.QtWidgets import (
    QApplication, QMainWindow, QFileDialog, QToolBar, QStatusBar,
    QWidget, QVBoxLayout, QHBoxLayout, QLabel, QPushButton,
    QDoubleSpinBox, QMessageBox, QGraphicsView, QGraphicsScene, QGridLayout, QTextEdit
)

# ---------------------------
# Constants
# ---------------------------
R = 6378137.0          # Web Mercator sphere radius (meters)
NM_TO_METERS = 1852.0  # nautical mile → meters

# ---------------------------
# Web Mercator helpers (lon/lat deg <-> meters)
# ---------------------------
def ll_to_merc(lon_deg: float, lat_deg: float) -> tuple[float, float]:
    lon = math.radians(lon_deg)
    lat = math.radians(lat_deg)
    lat = max(min(lat, math.radians(85.05112878)), math.radians(-85.05112878))
    x = R * lon
    y = R * math.log(math.tan(math.pi / 4.0 + lat / 2.0))
    return x, y

def merc_to_ll(x: float, y: float) -> tuple[float, float]:
    lon = math.degrees(x / R)
    lat = math.degrees(2.0 * math.atan(math.exp(y / R)) - math.pi / 2.0)
    return lon, lat

def bbox_xy(points_xy: List[tuple[float, float]]) -> tuple[float, float, float, float]:
    xs = [p[0] for p in points_xy]
    ys = [p[1] for p in points_xy]
    return min(xs), min(ys), max(xs), max(ys)

def center_xy(b: tuple[float, float, float, float]) -> tuple[float, float]:
    minx, miny, maxx, maxy = b
    return (minx + maxx) / 2.0, (miny + maxy) / 2.0

def clamp_center_shift_xy(route_bbox, box_bbox) -> tuple[float, float]:
    """Return (dx,dy) in meters that best centers the route inside the box (clamped)."""
    rminx, rminy, rmaxx, rmaxy = route_bbox
    bminx, bminy, bmaxx, bmaxy = box_bbox
    rcx, rcy = center_xy(route_bbox)
    bcx, bcy = center_xy(box_bbox)
    want_dx, want_dy = (bcx - rcx), (bcy - rcy)

    dx_lo, dx_hi = bminx - rminx, bmaxx - rmaxx
    dy_lo, dy_hi = bminy - rminy, bmaxy - rmaxy

    def pick(want, lo, hi):
        if lo <= hi:
            return min(max(want, lo), hi)  # feasible: clamp
        else:
            return (lo + hi) / 2.0         # infeasible (route bigger): minimize max overflow

    return pick(want_dx, dx_lo, dx_hi), pick(want_dy, dy_lo, dy_hi)

# ---------------------------
# JSON parsing (your shapes)
# ---------------------------
def read_polygon_points_ll(poly_json: Any) -> List[tuple[float, float]]:
    if not isinstance(poly_json, dict) or "polygon" not in poly_json:
        raise ValueError("Polygon JSON must include key 'polygon' (array of {lat, lon}).")
    arr = poly_json["polygon"]
    if not isinstance(arr, list) or not arr:
        raise ValueError("'polygon' must be a non-empty array.")
    pts = []
    for i, p in enumerate(arr):
        if not isinstance(p, dict) or "lat" not in p or "lon" not in p:
            raise ValueError(f"polygon[{i}] must have 'lat' and 'lon'.")
        pts.append((float(p["lon"]), float(p["lat"])))
    return pts

def read_waypoints_ll(mission_json: Any) -> List[tuple[list[Any], float, float]]:
    if not isinstance(mission_json, dict) or "WaypointMission" not in mission_json:
        raise ValueError("Mission JSON must contain 'WaypointMission'.")
    wm = mission_json["WaypointMission"]
    if not isinstance(wm, dict) or "waypoints" not in wm or not isinstance(wm["waypoints"], list):
        raise ValueError("'WaypointMission.waypoints' must be a list.")
    refs: List[tuple[list[Any], float, float]] = []
    for i, wpt in enumerate(wm["waypoints"]):
        if not isinstance(wpt, dict) or "loc" not in wpt or not isinstance(wpt["loc"], dict):
            raise ValueError(f"Waypoint {i} must have 'loc' object.")
        loc = wpt["loc"]
        if "lon" not in loc or "lat" not in loc:
            raise ValueError(f"Waypoint {i}.loc must have 'lon' and 'lat'.")
        refs.append((["WaypointMission", "waypoints", i, "loc"], float(loc["lon"]), float(loc["lat"])))
    return refs

def write_waypoints_from_ll(mission_json: Any, refs: List[tuple[list[Any], float, float]],
                            moved_ll: List[tuple[float, float]]) -> Any:
    out = json.loads(json.dumps(mission_json))
    for (path, _, _), (lon, lat) in zip(refs, moved_ll):
        cur = out
        for key in path[:-1]:
            cur = cur[key]
        cur[path[-1]]["lon"] = lon
        cur[path[-1]]["lat"] = lat
    return out

# ---------------------------
# Adapt (meters, Mercator)
# ---------------------------
def adapt_with_offsets_merc(poly_json: Any, mission_json: Any,
                            manual_dx_m: float, manual_dy_m: float):
    # read lon/lat
    box_ll = read_polygon_points_ll(poly_json)
    refs   = read_waypoints_ll(mission_json)
    route_ll = [(lon, lat) for _, lon, lat in refs]

    # project to meters
    box_xy   = [ll_to_merc(lon, lat) for lon, lat in box_ll]
    route_xy = [ll_to_merc(lon, lat) for lon, lat in route_ll]

    # bboxes (meters)
    box_bbox_xy   = bbox_xy(box_xy)
    route_bbox_xy = bbox_xy(route_xy)

    # auto center/clamp (meters) + manual offsets (meters)
    auto_dx, auto_dy = clamp_center_shift_xy(route_bbox_xy, box_bbox_xy)
    dx, dy = auto_dx + manual_dx_m, auto_dy + manual_dy_m

    # move and convert back
    moved_xy = [(x + dx, y + dy) for (x, y) in route_xy]
    moved_ll = [merc_to_ll(x, y) for (x, y) in moved_xy]

    adapted = write_waypoints_from_ll(mission_json, refs, moved_ll)
    return adapted, {"auto_dx": auto_dx, "auto_dy": auto_dy, "dx": dx, "dy": dy}, box_xy, moved_xy

# ---------------------------
# Zoomable & Draggable GraphicsView
# ---------------------------
class ZoomGraphicsView(QGraphicsView):
    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        # smooth transforms + zoom around cursor
        self.setRenderHints(self.renderHints())
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setResizeAnchor(QGraphicsView.AnchorViewCenter)
        self._zoom_min = -20  # ~1/(1.25^20)
        self._zoom_max =  20  # ~1.25^20
        self._zoom_level = 0
        self._auto_fit = True  # auto-fit until user zooms
        # dragging
        self._panning = False
        self.setDragMode(QGraphicsView.NoDrag)
        self.setCursor(Qt.ArrowCursor)

    # Mouse wheel zoom
    def wheelEvent(self, event):
        if event.modifiers() & Qt.ControlModifier:
            return super().wheelEvent(event)
        angle = event.angleDelta().y()
        if angle == 0:
            return
        step = 1 if angle > 0 else -1
        new_level = self._zoom_level + step
        if new_level < self._zoom_min or new_level > self._zoom_max:
            return
        factor = 1.25 if step > 0 else 1/1.25
        self.scale(factor, factor)
        self._zoom_level = new_level
        self._auto_fit = False  # stop auto-fitting after first manual zoom

    # Left-mouse drag to pan
    def mousePressEvent(self, event):
        if event.button() == Qt.LeftButton and not (event.modifiers() & Qt.ControlModifier):
            self._panning = True
            self.setDragMode(QGraphicsView.ScrollHandDrag)
            self.setCursor(Qt.ClosedHandCursor)
        super().mousePressEvent(event)

    def mouseReleaseEvent(self, event):
        if self._panning and event.button() == Qt.LeftButton:
            self._panning = False
            self.setDragMode(QGraphicsView.NoDrag)
            self.setCursor(Qt.ArrowCursor)
        super().mouseReleaseEvent(event)

    def reset_view(self):
        """Caller should fitInView(sceneRect) then reset zoom state."""
        self.resetTransform()
        self._zoom_level = 0
        self._auto_fit = True

    def should_auto_fit(self) -> bool:
        return self._auto_fit

# ---------------------------
# Preview (uniform scale, Mercator meters)
# ---------------------------
class Preview(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.view = ZoomGraphicsView()  # zoom + pan
        self.view.setMinimumHeight(240)
        self.scene = QGraphicsScene()
        self.view.setScene(self.scene)

        lay = QVBoxLayout(self)
        title = QLabel("Preview (Web Mercator meters; scroll to zoom, drag to pan)")
        title.setStyleSheet("color:#666; font-size:11px")
        lay.addWidget(title)
        lay.addWidget(self.view)

        self._last_scene_rect = QRectF(0, 0, 600, 500)

    def draw_xy(self, box_xy: Optional[List[tuple[float, float]]],
                route_xy: Optional[List[tuple[float, float]]]):
        self.scene.clear()
        if not box_xy and not route_xy:
            return

        # bbox + padding
        def _bbox(points):
            xs = [p[0] for p in points]; ys = [p[1] for p in points]
            return min(xs), min(ys), max(xs), max(ys)

        all_xy: List[tuple[float, float]] = []
        if box_xy:   all_xy += box_xy
        if route_xy: all_xy += route_xy

        minx, miny, maxx, maxy = _bbox(all_xy)
        padx = (maxx - minx) * 0.10 or 1.0
        pady = (maxy - miny) * 0.10 or 1.0
        minx -= padx; maxx += padx; miny -= pady; maxy += pady

        dx = maxx - minx; dy = maxy - miny
        W, H = 600.0, 500.0
        s = min(W / dx, H / dy)
        ox = (W - dx * s) / 2.0
        oy = (H - dy * s) / 2.0

        def to_px(xm: float, ym: float) -> tuple[float, float]:
            x = ox + (xm - minx) * s
            y = H - (oy + (ym - miny) * s)
            return x, y

        if box_xy:
            ring = box_xy + [box_xy[0]]
            pen = QPen(QColor("#888")); pen.setWidthF(1.5)
            for i in range(len(ring) - 1):
                x1, y1 = to_px(*ring[i]); x2, y2 = to_px(*ring[i + 1])
                self.scene.addLine(x1, y1, x2, y2, pen)

        if route_xy and len(route_xy) >= 1:
            pen_route = QPen(QColor("#1f77b4")); pen_route.setWidthF(2.0)
            for i in range(len(route_xy) - 1):
                x1, y1 = to_px(*route_xy[i]); x2, y2 = to_px(*route_xy[i + 1])
                self.scene.addLine(x1, y1, x2, y2, pen_route)
            pen_pt = QPen(QColor("#d62728")); brush_pt = QBrush(QColor("#d62728"))
            for x, y in route_xy:
                px, py = to_px(x, y); r = 3.0
                self.scene.addEllipse(px - r, py - r, 2 * r, 2 * r, pen_pt, brush_pt)

        # set a stable scene rect; only auto-fit if user hasn’t zoomed yet
        self._last_scene_rect = QRectF(0, 0, W, H)
        self.scene.setSceneRect(self._last_scene_rect)
        if self.view.should_auto_fit():
            self.view.reset_view()
            self.view.fitInView(self.scene.sceneRect(), Qt.KeepAspectRatio)

    def reset_view(self):
        """Explicitly refit view to content (used after center/reset or new files)."""
        self.view.reset_view()
        self.view.fitInView(self.scene.sceneRect(), Qt.KeepAspectRatio)

# ---------------------------
# Diff → HTML (highlight changed lines)
# ---------------------------
def json_to_pretty_lines(obj: Any) -> List[str]:
    return json.dumps(obj, indent=2, ensure_ascii=False).splitlines()

def html_highlight_changed(original_obj: Any, adapted_obj: Any) -> str:
    """
    Build HTML where lines that differ from original are highlighted green.
    """
    a = json_to_pretty_lines(original_obj)
    b = json_to_pretty_lines(adapted_obj)

    sm = difflib.SequenceMatcher(a=a, b=b, autojunk=False)
    changed_idx_b = set()
    for tag, i1, i2, j1, j2 in sm.get_opcodes():
        if tag == 'equal':
            continue
        for j in range(j1, j2):
            changed_idx_b.add(j)

    def esc(s: str) -> str:
        return (s.replace("&", "&amp;").replace("<", "&lt;")
                 .replace(">", "&gt;").replace("  ", "&nbsp;&nbsp;"))

    lines = []
    for idx, line in enumerate(b):
        text = esc(line)
        if idx in changed_idx_b:
            lines.append(f'<div style="background-color:#20C997;">{text}</div>')
        else:
            lines.append(f'<div>{text}</div>')
    return (
        '<pre style="background:transparent;'
        'white-space:pre;'
        'font-family:ui-monospace,SFMono-Regular,Menlo,Consolas,monospace;'
        'font-size:12px;margin:0;">'
        + "\n".join(lines) + "</pre>"
    )

# ---------------------------
# Main window
# ---------------------------
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.setWindowTitle("Waypoint Route → BBox Translator (Mercator) + N/S/E/W Cross")
        self.resize(1250, 820)

        # loaded data
        self.poly_json: Optional[Any] = None
        self.mission_json: Optional[Any] = None  # original mission (baseline)
        self.adapted_json: Optional[Any] = None

        # manual offsets in METERS (east = +x, north = +y)
        self.manual_dx_m = 0.0
        self.manual_dy_m = 0.0

        self.preview = Preview()
        self._build_ui()

    def _build_ui(self):
        tb = QToolBar("Main"); tb.setMovable(False); self.addToolBar(tb)
        tb.addAction(QAction("Open Polygon…", self, triggered=self.open_polygon))
        tb.addAction(QAction("Open Mission…", self, triggered=self.open_mission))
        tb.addSeparator()
        tb.addAction(QAction("Save Adapted…", self, triggered=self.save_adapted))

        self.status = QStatusBar(); self.setStatusBar(self.status)

        central = QWidget(); root = QVBoxLayout(central); self.setCentralWidget(central)

        self.poly_label = QLabel("Polygon: (none)")
        self.mission_label = QLabel("Mission: (none)")
        root.addWidget(self.poly_label); root.addWidget(self.mission_label)

        row = QHBoxLayout()
        left = QVBoxLayout()
        left.addWidget(self.preview)

        # Controls (top-right cross)
        controls = QVBoxLayout()
        controls.addWidget(QLabel("Nudge route by step (nautical miles), or center-reset:"))

        step_row = QHBoxLayout()
        self.step_nmi = QDoubleSpinBox()
        self.step_nmi.setDecimals(2)
        self.step_nmi.setSingleStep(0.1)
        self.step_nmi.setValue(0.10)
        step_row.addWidget(QLabel("Step:"))
        step_row.addWidget(self.step_nmi)
        step_row.addStretch(1)
        controls.addLayout(step_row)

        grid = QGridLayout()
        btn_n = QPushButton("N ↑")
        btn_s = QPushButton("S ↓")
        btn_e = QPushButton("E →")
        btn_w = QPushButton("W ←")
        btn_c = QPushButton("Center")
        for b in (btn_n, btn_s, btn_e, btn_w, btn_c):
            b.setMinimumWidth(100)
        grid.addWidget(QWidget(), 0, 0)
        grid.addWidget(btn_n,       0, 1)
        grid.addWidget(QWidget(), 0, 2)
        grid.addWidget(btn_w,       1, 0)
        grid.addWidget(btn_c,       1, 1)
        grid.addWidget(btn_e,       1, 2)
        grid.addWidget(QWidget(), 2, 0)
        grid.addWidget(btn_s,       2, 1)
        grid.addWidget(QWidget(), 2, 2)
        controls.addLayout(grid)

        # wiring
        btn_n.clicked.connect(lambda: self.nudge(0, +1))
        btn_s.clicked.connect(lambda: self.nudge(0, -1))
        btn_e.clicked.connect(lambda: self.nudge(+1, 0))
        btn_w.clicked.connect(lambda: self.nudge(-1, 0))
        btn_c.clicked.connect(self.center_reset)

        # Adapted Mission pane (larger)
        self.outputs_view = QTextEdit()
        self.outputs_view.setReadOnly(True)
        self.outputs_view.setLineWrapMode(QTextEdit.NoWrap)
        self.outputs_view.setMinimumHeight(420)

        left.addWidget(QLabel("Adapted Mission (changed lines highlighted)"))
        left.addWidget(self.outputs_view, 3)  # larger weight

        right = QVBoxLayout()
        right.addLayout(controls)
        right.addStretch(1)

        row.addLayout(left, 4)
        row.addLayout(right, 1)
        root.addLayout(row)

    # ---- File ops
    def open_polygon(self):
        path, _ = QFileDialog.getOpenFileName(self, "Open Polygon JSON", "", "JSON Files (*.json);;All Files (*)")
        if not path: return
        try:
            with open(path, "r", encoding="utf-8") as f:
                self.poly_json = json.load(f)
            _ = read_polygon_points_ll(self.poly_json)  # validate
            self.poly_label.setText(f"Polygon: {path}")
            self.status.showMessage("Loaded polygon JSON", 3000)
            self.redraw_preview_initial()
            self.preview.reset_view()
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to load polygon:\n{e}")

    def open_mission(self):
        path, _ = QFileDialog.getOpenFileName(self, "Open Waypoint Mission JSON", "", "JSON Files (*.json);;All Files (*)")
        if not path: return
        try:
            with open(path, "r", encoding="utf-8") as f:
                self.mission_json = json.load(f)
            _ = read_waypoints_ll(self.mission_json)  # validate
            self.mission_label.setText(f"Mission: {path}")
            # reset manual meters and center immediately
            self.manual_dx_m = 0.0; self.manual_dy_m = 0.0
            self.apply_and_render()
            self.preview.reset_view()
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to load mission:\n{e}")

    # ---- Core actions
    def apply_and_render(self):
        if self.poly_json is None or self.mission_json is None:
            return
        try:
            adapted, xform, box_xy, moved_xy = adapt_with_offsets_merc(
                self.poly_json, self.mission_json, self.manual_dx_m, self.manual_dy_m
            )
            self.adapted_json = adapted
            self.outputs_view.setHtml(html_highlight_changed(self.mission_json, self.adapted_json))
            self.preview.draw_xy(box_xy, moved_xy)
            msg = (f"Auto dx={xform['auto_dx']:.1f} m, dy={xform['auto_dy']:.1f} m | "
                   f"Manual dx={self.manual_dx_m:.1f} m, dy={self.manual_dy_m:.1f} m")
            self.status.showMessage(msg, 6000)
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Adapt failed:\n{e}")

    def nudge(self, east_sign: int, north_sign: int):
        if self.poly_json is None or self.mission_json is None:
            QMessageBox.warning(self, "Warning", "Load both Polygon and Mission JSON.")
            return
        step_m = float(self.step_nmi.value()) * NM_TO_METERS
        self.manual_dx_m += east_sign  * step_m  # +east
        self.manual_dy_m += north_sign * step_m  # +north
        self.apply_and_render()

    def center_reset(self):
        """Center the route inside the op box (clear manual offsets, re-apply)."""
        self.manual_dx_m = 0.0
        self.manual_dy_m = 0.0
        self.apply_and_render()
        self.preview.reset_view()

    # ---- Save
    def save_adapted(self):
        if self.adapted_json is None:
            QMessageBox.warning(self, "Warning", "Nothing to save — load inputs first.")
            return
        path, _ = QFileDialog.getSaveFileName(self, "Save Adapted Mission",
                                              "waypoint_mission_adapted.json",
                                              "JSON Files (*.json);;All Files (*)")
        if not path: return
        try:
            with open(path, "w", encoding="utf-8") as f:
                json.dump(self.adapted_json, f, indent=2, ensure_ascii=False)
            self.status.showMessage(f"Saved: {path}", 5000)
        except Exception as e:
            QMessageBox.critical(self, "Error", f"Failed to save adapted route:\n{e}")

    # ---- Helpers
    def redraw_preview_initial(self):
        try:
            box_xy = None
            route_xy = None
            if self.poly_json is not None:
                box_ll = read_polygon_points_ll(self.poly_json)
                box_xy = [ll_to_merc(lon, lat) for lon, lat in box_ll]
            if self.mission_json is not None:
                refs = read_waypoints_ll(self.mission_json)
                route_ll = [(lon, lat) for _, lon, lat in refs]
                route_xy = [ll_to_merc(lon, lat) for lon, lat in route_ll]
            if box_xy or route_xy:
                self.preview.draw_xy(box_xy, route_xy)
        except Exception:
            pass

# ---------------------------
# Main
# ---------------------------
def main():
    app = QApplication(sys.argv)
    win = MainWindow()
    win.show()
    sys.exit(app.exec())

if __name__ == "__main__":
    main()
