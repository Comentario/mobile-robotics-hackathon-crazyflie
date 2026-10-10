"""English live mission dashboard. No radio access or flight commands."""
class StatusPanel:
    def __init__(self):
        from PyQt6.QtWidgets import QApplication, QWidget, QVBoxLayout, QLabel, QProgressBar
        self.app = QApplication.instance() or QApplication([])
        self.window = QWidget()
        self.window.setWindowTitle('CF2 · Live mapping status')
        self.window.resize(440, 260)
        layout = QVBoxLayout(self.window)
        self.status = QLabel('Waiting for telemetry')
        self.progress = QProgressBar(); self.progress.setRange(0, 1000)
        self.progress.setFormat('Observed area: %p%')
        self.details = QLabel(); self.details.setWordWrap(True)
        for widget in (self.status, self.progress, self.details): layout.addWidget(widget)
        self.window.show()

    def update(self, snapshot):
        nav = snapshot.get('navigation') or {}
        coverage = nav.get('coverage') or {}
        age = snapshot.get('age_seconds')
        live = 'WAITING' if age is None else ('LIVE' if age <= 1 else 'STALE DATA')
        self.status.setText(f"{live} · {coverage.get('phase', 'WAITING')}")
        percent = coverage.get('observed_percent', 0)
        self.progress.setValue(round(percent*10))
        self.details.setText(
            f"Observed: {percent:.1f}% · Target: {coverage.get('target_percent', '—')}%\n"
            f"Unknown cells: {coverage.get('unknown_cells', '—')}\n"
            f"Obstacle components: {len(coverage.get('obstacles', []))} (partial observations)\n"
            f"Visited viewpoints: {len(coverage.get('visited', []))}\n"
            f"Stop trigger: {coverage.get('finish_trigger') or 'Exploration in progress'}\n"
            'Coverage measures observations; it does not certify obstacle contours or flight safety.')
        self.app.processEvents()
