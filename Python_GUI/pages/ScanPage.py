import asyncio
import os
import sys
import threading

from typing import List, Tuple

try:
    from PySide6 import QtCore, QtWidgets
except ImportError as e:
    raise SystemExit("PySide6 is required. Install with: pip install PySide6") from e

from services import QtExoDeviceManager
from utils import UIConfig, load_logo, style_button, apply_button_style_batch


UART_SERVICE_UUID = "6E400001-B5A3-F393-E0A9-E50E24DCCA9E"  # Nordic UART Service
BOOT_CONFIG_KEY_COUNT = 84
BOOT_CONFIG_KEY_LABELS = [
    "Board Name",
    "Board Version",
    "Battery",
    "Exo Name",
    "Exo Side",
    "Hip Motor",
    "Knee Motor",
    "Ankle Motor",
    "Elbow Motor",
    "Arm 1 Motor",
    "Arm 2 Motor",
    "Hip Gear Ratio",
    "Knee Gear Ratio",
    "Ankle Gear Ratio",
    "Elbow Gear Ratio",
    "Arm 1 Gear Ratio",
    "Arm 2 Gear Ratio",
    "Hip Default Controller",
    "Knee Default Controller",
    "Ankle Default Controller",
    "Elbow Default Controller",
    "Arm 1 Default Controller",
    "Arm 2 Default Controller",
    "Hip Use Torque Sensor",
    "Knee Use Torque Sensor",
    "Ankle Use Torque Sensor",
    "Elbow Use Torque Sensor",
    "Arm 1 Use Torque Sensor",
    "Arm 2 Use Torque Sensor",
    "Hip Flip Motor Dir",
    "Knee Flip Motor Dir",
    "Ankle Flip Motor Dir",
    "Elbow Flip Motor Dir",
    "Arm 1 Flip Motor Dir",
    "Arm 2 Flip Motor Dir",
    "Hip Flip Torque Dir",
    "Knee Flip Torque Dir",
    "Ankle Flip Torque Dir",
    "Elbow Flip Torque Dir",
    "Arm 1 Flip Torque Dir",
    "Arm 2 Flip Torque Dir",
    "Hip Flip Angle Dir",
    "Knee Flip Angle Dir",
    "Ankle Flip Angle Dir",
    "Elbow Flip Angle Dir",
    "Arm 1 Flip Angle Dir",
    "Arm 2 Flip Angle Dir",
    "Left Hip RoM",
    "Right Hip RoM",
    "Left Knee RoM",
    "Right Knee RoM",
    "Left Ankle RoM",
    "Right Ankle RoM",
    "Left Elbow RoM",
    "Right Elbow RoM",
    "Left Arm 1 RoM",
    "Right Arm 1 RoM",
    "Left Arm 2 RoM",
    "Right Arm 2 RoM",
    "Left Hip Torque Offset",
    "Right Hip Torque Offset",
    "Left Knee Torque Offset",
    "Right Knee Torque Offset",
    "Left Ankle Torque Offset",
    "Right Ankle Torque Offset",
    "Left Elbow Torque Offset",
    "Right Elbow Torque Offset",
    "Left Arm 1 Torque Offset",
    "Right Arm 1 Torque Offset",
    "Left Arm 2 Torque Offset",
    "Right Arm 2 Torque Offset",
    "Max Sensor Torque Rate",
    "Max Sensor Torque Rate Cycle Limit",
    "Max Sensor Torque",
    "Max Sensor Torque Cycle Limit",
    "Max Desired Torque",
    "Max Desired Torque Cycle Limit",
    "Max Desired Torque Rate",
    "Max Desired Torque Rate Cycle Limit",
    "Max Driver Torque",
    "Max Driver Torque Cycle Limit",
    "Max Driver Torque Rate",
    "Max Driver Torque Rate Cycle Limit",
    "Static Driver Torque Cycle Limit",
]

# Encoded defaults aligned to SDCard/config.ini in config key index order (0..83).
BOOT_CONFIG_DEFAULT_VALUES = [
    1, 1, 1, 1, 1, 1, 1, 6, 1, 1, 1, 1, 1, 1, 1, 1, 1,
    1, 1, 3, 1, 1, 1, 1, 1, 2, 1, 1, 1,
    1, 1, 2, 1, 1, 1,
    1, 1, 3, 1, 1, 1,
    1, 1, 1, 1, 1, 1,
    0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0,
    255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255, 255,
    5, 50, 35, 50, 21, 50, 5, 50, 35, 50, 6, 50, 50,
]

BOOT_CONFIG_ENUM_OPTIONS = {
    3: [
        ("Ankle", 1), ("Hip", 2), ("Knee", 3), ("Elbow", 4),
        ("HipAnkle", 5), ("HipElbow", 6), ("AnkleElbow", 7),
        ("leftAnkle", 8), ("rightAnkle", 9), ("leftHip", 10), ("rightHip", 11),
        ("leftKnee", 12), ("rightKnee", 13), ("leftElbow", 14), ("rightElbow", 15),
        ("leftHipAnkle", 16), ("rightHipAnkle", 17), ("leftHipElbow", 18), ("rightHipElbow", 19),
        ("leftAnkleElbow", 20), ("rightAnkleElbow", 21), ("test", 22), ("Arm", 23),
    ],
    4: [("bilateral", 1), ("left", 2), ("right", 3)],
}

_MOTOR_OPTIONS = [("0", 1), ("AK60", 2), ("AK80", 3), ("AK60v1.1", 4), ("AK70", 5), ("MaxonMotor", 6), ("NullMotor", 7), ("AK60v3", 8), ("AK45_36", 9), ("AK45_10", 10)]
for _i in range(5, 11):
    BOOT_CONFIG_ENUM_OPTIONS[_i] = _MOTOR_OPTIONS

_GEAR_OPTIONS = [("1", 1), ("2", 2), ("3", 3), ("4.5", 4)]
for _i in range(11, 17):
    BOOT_CONFIG_ENUM_OPTIONS[_i] = _GEAR_OPTIONS

BOOT_CONFIG_ENUM_OPTIONS[17] = [("0", 1), ("zeroTorque", 2), ("franksCollinsHip", 3), ("constantTorque", 4), ("chirp", 5), ("step", 6), ("phmc", 7), ("calibrManager", 8), ("spline", 9)]
BOOT_CONFIG_ENUM_OPTIONS[18] = [("0", 1), ("zeroTorque", 2), ("constantTorque", 3), ("chirp", 4), ("step", 5), ("calibrManager", 6)]
BOOT_CONFIG_ENUM_OPTIONS[19] = [("0", 1), ("zeroTorque", 2), ("PJMC", 3), ("zhangCollins", 4), ("constantTorque", 5), ("TREC", 6), ("calibrManager", 7), ("chirp", 8), ("step", 9), ("SPV2", 10), ("PJMC_PLUS", 11), ("spline", 12)]
BOOT_CONFIG_ENUM_OPTIONS[20] = [("0", 1), ("zeroTorque", 2), ("elbowMinMax", 3), ("calibrManager", 4), ("chirp", 5), ("step", 6)]
BOOT_CONFIG_ENUM_OPTIONS[21] = [("0", 1), ("zeroTorque", 2), ("constantTorque", 3), ("spline", 4)]
BOOT_CONFIG_ENUM_OPTIONS[22] = [("0", 1), ("zeroTorque", 2), ("constantTorque", 3), ("spline", 4)]

_USE_TORQUE_OPTIONS = [("0", 1), ("yes", 2)]
for _i in range(23, 29):
    BOOT_CONFIG_ENUM_OPTIONS[_i] = _USE_TORQUE_OPTIONS

_FLIP_OPTIONS = [("0", 1), ("left", 2), ("right", 3), ("both", 4)]
for _i in range(29, 47):
    BOOT_CONFIG_ENUM_OPTIONS[_i] = _FLIP_OPTIONS


class DeviceScannerWorker(QtCore.QObject):
    resultsReady = QtCore.Signal(list)  # List[Tuple[str, str]] (name, address)
    error = QtCore.Signal(str)

    def __init__(self, qt_dev: QtExoDeviceManager):
        super().__init__()
        self._qt_dev = qt_dev
        self._qt_dev.scanResults.connect(self._forward)
        self._qt_dev.error.connect(self.error)

    @QtCore.Slot()
    def scan_once(self):
        self._qt_dev.scan()

    @QtCore.Slot(list)
    def _forward(self, results):
        self.resultsReady.emit(results)


class OverwriteBootConfigDialog(QtWidgets.QDialog):
    keyFutureDone = QtCore.Signal(object)
    configFutureDone = QtCore.Signal(object)
    queryFutureDone = QtCore.Signal(object)

    def __init__(self, qt_dev: QtExoDeviceManager, parent=None):
        super().__init__(parent)
        self._qt_dev = qt_dev
        self._entries: list[QtWidgets.QWidget | None] = []
        self._pending_pairs: list[tuple[int, int]] = []
        self._current_index = 0
        self._awaiting_key_ack = False
        self._query_inflight = False
        self._query_values: list[int | None] = [None] * BOOT_CONFIG_KEY_COUNT
        self._pre_query_values: list[int] = []
        self._query_timeout_ms = 15000
        self._query_retry_count = 0
        self._query_max_retries = 2
        self._key_ack_timeout_ms = 3000

        self._query_timeout_timer = QtCore.QTimer(self)
        self._query_timeout_timer.setSingleShot(True)
        self._query_timeout_timer.timeout.connect(self._on_query_timeout)

        self._key_ack_timeout_timer = QtCore.QTimer(self)
        self._key_ack_timeout_timer.setSingleShot(True)
        self._key_ack_timeout_timer.timeout.connect(self._on_key_ack_timeout)

        self.setWindowTitle("Overwrite Boot Config")
        self.setModal(True)
        self.resize(540, 620)

        root = QtWidgets.QVBoxLayout(self)
        root.setContentsMargins(UIConfig.MARGIN_PAGE, UIConfig.MARGIN_PAGE, UIConfig.MARGIN_PAGE, UIConfig.MARGIN_PAGE)
        root.setSpacing(UIConfig.SPACING_SMALL)

        title = QtWidgets.QLabel("Overwrite Boot Config")
        title_font = title.font()
        title_font.setPointSize(UIConfig.FONT_LARGE)
        title.setFont(title_font)
        title.setAlignment(QtCore.Qt.AlignCenter)
        root.addWidget(title)

        subtitle = QtWidgets.QLabel("Set values for all config keys (0-255), then click Overwrite Boot Config.")
        subtitle.setWordWrap(True)
        subtitle.setAlignment(QtCore.Qt.AlignCenter)
        root.addWidget(subtitle)

        self.status = QtWidgets.QLabel("Ready")
        self.status.setAlignment(QtCore.Qt.AlignCenter)
        root.addWidget(self.status)

        self.progress = QtWidgets.QProgressBar()
        self.progress.setRange(0, BOOT_CONFIG_KEY_COUNT + 1)
        self.progress.setValue(0)
        root.addWidget(self.progress)

        scroll = QtWidgets.QScrollArea()
        scroll.setWidgetResizable(True)
        scroll_content = QtWidgets.QWidget()
        form = QtWidgets.QFormLayout(scroll_content)
        form.setLabelAlignment(QtCore.Qt.AlignRight)
        form.setFormAlignment(QtCore.Qt.AlignTop)

        hidden_key_indices = {0, 1, 2}  # board_name, board_version, battery (backend-only)

        for key_index in range(BOOT_CONFIG_KEY_COUNT):
            default_value = BOOT_CONFIG_DEFAULT_VALUES[key_index] if key_index < len(BOOT_CONFIG_DEFAULT_VALUES) else 0
            if key_index in hidden_key_indices:
                self._entries.append(None)
                continue

            control = self._create_entry_control(key_index, default_value)
            self._entries.append(control)
            key_label = BOOT_CONFIG_KEY_LABELS[key_index] if key_index < len(BOOT_CONFIG_KEY_LABELS) else f"Key {key_index}"
            form.addRow(key_label, control)

        scroll.setWidget(scroll_content)
        root.addWidget(scroll, 1)

        btn_row = QtWidgets.QHBoxLayout()
        btn_row.setSpacing(UIConfig.SPACING_MEDIUM)
        self.btn_cancel = QtWidgets.QPushButton("Cancel")
        self.btn_query = QtWidgets.QPushButton("Query Config")
        self.btn_overwrite = QtWidgets.QPushButton("Overwrite Boot Config")
        btn_row.addWidget(self.btn_cancel)
        btn_row.addWidget(self.btn_query)
        btn_row.addWidget(self.btn_overwrite)
        root.addLayout(btn_row)

        apply_button_style_batch(
            [self.btn_cancel, self.btn_query, self.btn_overwrite],
            height=UIConfig.BTN_HEIGHT_MEDIUM,
            width=UIConfig.BTN_WIDTH_SMALL,
            font_size=UIConfig.FONT_MEDIUM,
            padding="6px 12px"
        )

        self.btn_cancel.clicked.connect(self.reject)
        self.btn_query.clicked.connect(self._on_query_clicked)
        self.btn_overwrite.clicked.connect(self._on_overwrite_clicked)
        self.keyFutureDone.connect(self._handle_key_done)
        self.configFutureDone.connect(self._handle_overwrite_config_done)
        self.queryFutureDone.connect(self._handle_query_done)

    @QtCore.Slot()
    def _on_overwrite_clicked(self):
        self._query_inflight = False
        self._query_timeout_timer.stop()
        self._key_ack_timeout_timer.stop()
        self._awaiting_key_ack = False
        self._set_busy(True)
        self._pending_pairs = [(index, self._entry_value(index)) for index in range(len(self._entries))]
        self._current_index = 0
        self.progress.setValue(0)
        self.status.setText("Sending key/value pairs...")
        self._send_next_key()

    @QtCore.Slot()
    def _on_query_clicked(self):
        self._pre_query_values = [self._entry_value(index) for index in range(len(self._entries))]
        self._query_values = [None] * BOOT_CONFIG_KEY_COUNT
        self._query_inflight = True
        self._query_retry_count = 0
        self._awaiting_key_ack = False
        self._key_ack_timeout_timer.stop()
        self._set_busy(True)
        self._set_loading_state(True)
        self.progress.setRange(0, BOOT_CONFIG_KEY_COUNT)
        self.progress.setValue(0)
        self.status.setText("LOADING config from device...")

        fut = self._qt_dev.queryConfigFuture()
        if fut is None:
            self._fail("Device is not connected")
            return
        fut.add_done_callback(lambda f: self.queryFutureDone.emit(f))

    def _create_entry_control(self, key_index: int, default_value: int) -> QtWidgets.QWidget:
        options = BOOT_CONFIG_ENUM_OPTIONS.get(key_index)
        if not options:
            spin = QtWidgets.QSpinBox()
            spin.setRange(0, 255)
            spin.setValue(default_value)
            return spin

        combo = QtWidgets.QComboBox()
        for label, encoded_value in options:
            combo.addItem(f"{label} ({encoded_value})", encoded_value)

        match_index = combo.findData(default_value)
        if match_index >= 0:
            combo.setCurrentIndex(match_index)
        else:
            combo.addItem(f"raw ({default_value})", default_value)
            combo.setCurrentIndex(combo.count() - 1)
        return combo

    def _entry_value(self, key_index: int) -> int:
        widget = self._entries[key_index]
        if widget is None:
            return BOOT_CONFIG_DEFAULT_VALUES[key_index] if key_index < len(BOOT_CONFIG_DEFAULT_VALUES) else 0
        if isinstance(widget, QtWidgets.QComboBox):
            data = widget.currentData()
            return int(data) if data is not None else 0
        if isinstance(widget, QtWidgets.QSpinBox):
            return int(widget.value())
        return 0

    def _set_busy(self, is_busy: bool):
        self.btn_cancel.setEnabled(not is_busy)
        self.btn_query.setEnabled(not is_busy)
        self.btn_overwrite.setEnabled(not is_busy)
        for entry in self._entries:
            if entry is not None:
                entry.setEnabled(not is_busy)

    def _set_loading_state(self, loading: bool):
        for key_index, entry in enumerate(self._entries):
            if entry is None:
                continue
            if isinstance(entry, QtWidgets.QComboBox):
                if loading:
                    entry.clear()
                    entry.addItem("LOADING...", None)
                    entry.setCurrentIndex(0)
                entry.setEnabled(not loading)
            elif isinstance(entry, QtWidgets.QSpinBox):
                if loading:
                    entry.setSpecialValueText("LOADING")
                    entry.setValue(0)
                entry.setEnabled(not loading)

    def _set_entry_value(self, key_index: int, value: int):
        entry = self._entries[key_index]
        if entry is None:
            return

        if isinstance(entry, QtWidgets.QComboBox):
            entry.clear()
            options = BOOT_CONFIG_ENUM_OPTIONS.get(key_index)
            if options:
                for label, encoded_value in options:
                    entry.addItem(f"{label} ({encoded_value})", encoded_value)
                match_index = entry.findData(value)
                if match_index >= 0:
                    entry.setCurrentIndex(match_index)
                else:
                    entry.addItem(f"raw ({value})", value)
                    entry.setCurrentIndex(entry.count() - 1)
            else:
                entry.addItem(str(value), value)
                entry.setCurrentIndex(0)
            entry.setEnabled(True)
            return

        if isinstance(entry, QtWidgets.QSpinBox):
            entry.setSpecialValueText("")
            entry.setValue(int(value))
            entry.setEnabled(True)

    def _apply_values(self, values: list[int]):
        for key_index in range(min(len(values), len(self._entries))):
            self._set_entry_value(key_index, int(values[key_index]))

    def _send_next_key(self):
        if self._current_index >= len(self._pending_pairs):
            self._awaiting_key_ack = False
            self.status.setText("All key/value pairs sent. Sending overwrite_config...")
            fut = self._qt_dev.overwriteConfigFuture()
            if fut is None:
                self._fail("Device is not connected")
                return
            fut.add_done_callback(lambda f: self.configFutureDone.emit(f))
            return

        key_index, key_value = self._pending_pairs[self._current_index]
        self.status.setText(f"Sending key {key_index}...")
        fut = self._qt_dev.overwriteKeyFuture(float(key_index), float(key_value))
        if fut is None:
            self._fail("Device is not connected")
            return
        fut.add_done_callback(lambda f: self.keyFutureDone.emit(f))

    def _handle_key_done(self, future):
        try:
            ok = future.result()
            if not ok:
                self._fail(f"Failed to send key {self._current_index}: timeout or write failure")
                return
        except Exception as ex:
            self._fail(f"Failed to send key {self._current_index}: {ex}")
            return

        if self._current_index >= len(self._pending_pairs):
            self._fail("Internal overwrite state error")
            return

        key_index, _ = self._pending_pairs[self._current_index]
        self._awaiting_key_ack = True
        self._key_ack_timeout_timer.start(self._key_ack_timeout_ms)
        self.status.setText(f"Waiting for key {key_index} acknowledgement...")

    def _handle_overwrite_config_done(self, future):
        try:
            future.result()
        except Exception as ex:
            self._fail(f"Failed to send overwrite_config: {ex}")
            return

        self.progress.setValue(BOOT_CONFIG_KEY_COUNT + 1)
        self.status.setText("Overwrite Boot Config completed")
        QtWidgets.QMessageBox.information(self, "Success", "Boot config overwrite completed successfully.")
        self.accept()

    def _handle_query_done(self, future):
        try:
            future.result()
        except Exception as ex:
            self._fail(f"Failed to send query_config: {ex}")
            return

        self.status.setText(
            f"LOADING config chunks... (attempt {self._query_retry_count + 1}/{self._query_max_retries + 1})"
        )
        self._query_timeout_timer.start(self._query_timeout_ms)

    @QtCore.Slot(list)
    def on_config_chunk_received(self, values):
        if not self._query_inflight:
            return
        if not values or len(values) < 2:
            return

        try:
            start_idx = int(round(float(values[0])))
            count = int(round(float(values[1])))
        except Exception:
            return

        if start_idx < 0 or start_idx >= BOOT_CONFIG_KEY_COUNT or count <= 0:
            return

        max_count = min(count, BOOT_CONFIG_KEY_COUNT - start_idx, max(0, len(values) - 2))
        for offset in range(max_count):
            raw_value = int(round(float(values[offset + 2])))
            clamped_value = max(0, min(255, raw_value))
            self._query_values[start_idx + offset] = clamped_value

        loaded_count = sum(1 for val in self._query_values if val is not None)
        self.progress.setValue(loaded_count)
        self._query_timeout_timer.start(self._query_timeout_ms)

        if loaded_count >= BOOT_CONFIG_KEY_COUNT:
            self._query_timeout_timer.stop()
            final_values = [
                int(val) if val is not None else BOOT_CONFIG_DEFAULT_VALUES[idx]
                for idx, val in enumerate(self._query_values)
            ]
            self._apply_values(final_values)
            self._query_inflight = False
            self._set_loading_state(False)
            self._set_busy(False)
            self.status.setText("Query complete")

    @QtCore.Slot(list)
    def on_overwrite_key_ack_received(self, values):
        if not self._awaiting_key_ack:
            return
        if self._current_index >= len(self._pending_pairs):
            return
        if not values or len(values) < 2:
            return

        try:
            key_index = int(round(float(values[0])))
            status = int(round(float(values[1])))
        except Exception:
            return

        expected_key_index, _ = self._pending_pairs[self._current_index]
        if key_index != expected_key_index:
            return

        self._key_ack_timeout_timer.stop()
        self._awaiting_key_ack = False

        if status != 1:
            self._fail(f"Device rejected key {key_index}")
            return

        self._current_index += 1
        self.progress.setValue(self._current_index)
        self._send_next_key()

    def _on_query_timeout(self):
        if not self._query_inflight:
            return
        loaded_count = sum(1 for val in self._query_values if val is not None)
        if loaded_count >= BOOT_CONFIG_KEY_COUNT:
            return

        if self._query_retry_count < self._query_max_retries:
            self._query_retry_count += 1
            self.status.setText(
                f"Partial config received ({loaded_count}/{BOOT_CONFIG_KEY_COUNT}). Retrying..."
            )
            fut = self._qt_dev.queryConfigFuture()
            if fut is None:
                self._fail("Device is not connected")
                return
            fut.add_done_callback(lambda f: self.queryFutureDone.emit(f))
            return

        self._fail(f"Timed out while loading config ({loaded_count}/{BOOT_CONFIG_KEY_COUNT} keys)")

    def _on_key_ack_timeout(self):
        if not self._awaiting_key_ack:
            return
        if self._current_index >= len(self._pending_pairs):
            self._fail("Timed out waiting for overwrite acknowledgement")
            return
        key_index, _ = self._pending_pairs[self._current_index]
        self._fail(f"Timed out waiting for key {key_index} acknowledgement")

    def _fail(self, message: str):
        self._query_timeout_timer.stop()
        self._key_ack_timeout_timer.stop()
        self._awaiting_key_ack = False
        if self._query_inflight:
            self._query_inflight = False
            self._set_loading_state(False)
            if self._pre_query_values:
                self._apply_values(self._pre_query_values)
        self._set_busy(False)
        self.status.setText(message)
        QtWidgets.QMessageBox.critical(self, "Overwrite Boot Config Failed", message)


class ScanWindowQt(QtWidgets.QWidget):
    # Signal to request a connection via the Qt device manager (address)
    connectRequested = QtCore.Signal(str)

    def __init__(self):
        super().__init__()
        self.setWindowTitle("OpenExo - Scan (Qt)")
        # Compact default size (resizable)
        self.setMinimumSize(UIConfig.WINDOW_MIN_WIDTH, UIConfig.WINDOW_MIN_HEIGHT)
        self.resize(UIConfig.WINDOW_DEFAULT_WIDTH, UIConfig.WINDOW_DEFAULT_HEIGHT)
        
        # Settings file path
        base_dir = os.path.dirname(os.path.dirname(__file__))
        self.SETTINGS_FILE = os.path.join(base_dir, "Saved_Data", "saved_device.txt")

        self.selected_address: str | None = None
        self.selected_name: str | None = None
        self._qt_dev: QtExoDeviceManager | None = None
        self._scanner: DeviceScannerWorker | None = None
        self._connected = False
        self._pending_scan = False
        self._active_overwrite_dialog: OverwriteBootConfigDialog | None = None

        self._build_ui()
        self._wire_workers()
        self._load_saved_device()

    def _build_ui(self):
        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(UIConfig.MARGIN_PAGE, 0, UIConfig.MARGIN_PAGE, 3)
        layout.setSpacing(0)  # Manual spacing control
        
        # Add spacing at the top
        layout.addSpacing(UIConfig.SPACING_SECTION)

        # Header row with logos and centered title
        header_row = QtWidgets.QHBoxLayout()
        header_row.setContentsMargins(0, 0, 0, 0)
        header_row.setSpacing(UIConfig.SPACING_SMALL)
        
        # Add OpenExo logo at left
        openexo_logo = load_logo(
            "OpenExo.png",
            UIConfig.LOGO_OPENEXO_WIDTH,
            UIConfig.LOGO_OPENEXO_HEIGHT,
            QtCore.Qt.AlignLeft | QtCore.Qt.AlignVCenter
        )
        if openexo_logo:
            header_row.addWidget(openexo_logo)
        
        # Add centered title
        self.title = QtWidgets.QLabel("OpenExo GUI - Qt Scan")
        self.title.setAlignment(QtCore.Qt.AlignCenter)
        self.title.setContentsMargins(0, 0, 0, 0)
        font = self.title.font()
        font.setPointSize(UIConfig.FONT_TITLE_LARGE)
        self.title.setFont(font)
        header_row.addWidget(self.title, 1)  # Stretch factor to center
        
        # Add Lab logo at right
        lab_logo = load_logo(
            "LabLogo.png",
            UIConfig.LOGO_LAB_WIDTH,
            UIConfig.LOGO_LAB_HEIGHT,
            QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter
        )
        if lab_logo:
            header_row.addWidget(lab_logo)
        
        layout.addLayout(header_row)
        layout.addSpacing(UIConfig.SPACING_HEADER)  # Spacing between header and status

        self.status = QtWidgets.QLabel("Not Connected")
        self.status.setAlignment(QtCore.Qt.AlignCenter)
        self.status.setContentsMargins(0, 0, 0, 0)
        f2 = self.status.font(); f2.setPointSize(UIConfig.FONT_MEDIUM); self.status.setFont(f2)
        layout.addWidget(self.status)
        layout.addSpacing(UIConfig.MARGIN_PAGE)  # Spacing between status and buttons

        # Button row
        btn_row = QtWidgets.QHBoxLayout()
        btn_row.setSpacing(UIConfig.SPACING_MEDIUM)
        btn_row.setContentsMargins(0, 0, 0, 0)
        self.btn_scan = QtWidgets.QPushButton("1. Start Scan")
        self.btn_load = QtWidgets.QPushButton("Load Saved Device")
        self.btn_overwrite_boot = QtWidgets.QPushButton("Overwrite Boot Config")
        self.btn_overwrite_boot.setEnabled(False)
        btn_row.addWidget(self.btn_scan)
        btn_row.addWidget(self.btn_load)
        btn_row.addWidget(self.btn_overwrite_boot)
        layout.addLayout(btn_row)
        layout.addSpacing(UIConfig.SPACING_TINY)

        # Devices list - fills space between top and bottom buttons
        self.list_devices = QtWidgets.QListWidget()
        self.list_devices.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        lf = self.list_devices.font(); lf.setPointSize(UIConfig.FONT_MEDIUM); self.list_devices.setFont(lf)
        self.list_devices.setStyleSheet(f"QListWidget::item{{ height: {UIConfig.LIST_ITEM_HEIGHT}px; padding: 3px 6px; }}")
        self.list_devices.setMinimumHeight(UIConfig.LIST_DEVICE_MIN_HEIGHT)
        layout.addWidget(self.list_devices, 1)  # Stretch factor 1 to fill space
        
        layout.addSpacing(5)

        # Action row (Connect, Calibrate Torque, Start Trial) - at bottom
        action_row = QtWidgets.QHBoxLayout()
        action_row.setSpacing(UIConfig.SPACING_MEDIUM)
        action_row.setContentsMargins(0, 0, 0, 0)
        self.btn_save_connect = QtWidgets.QPushButton("2. Connect")
        self.btn_save_connect.setEnabled(False)  # Disabled initially
        self.btn_start_trial = QtWidgets.QPushButton("4. Start Trial")
        self.btn_start_trial.setEnabled(False)
        self.btn_calibrate_torque = QtWidgets.QPushButton("3. Calibrate Torque")
        self.btn_calibrate_torque.setEnabled(False)
        action_row.addWidget(self.btn_save_connect)
        action_row.addWidget(self.btn_calibrate_torque)
        action_row.addWidget(self.btn_start_trial)
        layout.addLayout(action_row)
        
        # Add spacing at the bottom
        layout.addSpacing(UIConfig.SPACING_SECTION)

        # Signals
        self.btn_scan.clicked.connect(self.on_scan)
        self.btn_load.clicked.connect(self.on_load_saved)
        self.btn_overwrite_boot.clicked.connect(self.on_overwrite_boot_config)
        self.btn_save_connect.clicked.connect(self.on_save_and_connect)
        self.btn_calibrate_torque.clicked.connect(self.on_calibrate_torque)
        self.list_devices.itemSelectionChanged.connect(self.on_selected)

        # Apply consistent button styling
        buttons = [self.btn_scan, self.btn_load, self.btn_overwrite_boot, self.btn_save_connect, self.btn_start_trial, self.btn_calibrate_torque]
        apply_button_style_batch(
            buttons,
            height=UIConfig.BTN_HEIGHT_MEDIUM,
            width=UIConfig.BTN_WIDTH_SMALL,
            font_size=UIConfig.FONT_MEDIUM,
            padding="6px 12px"
        )
        # Lock max height for compact layout
        for btn in buttons:
            btn.setMaximumHeight(UIConfig.BTN_HEIGHT_MEDIUM)

    def _wire_workers(self):
        # Use the shared QtExoDeviceManager instance provided by MainWindow after construction
        # We lazy-bind scanner to the device manager via bind_device_manager()
        self._scanner: DeviceScannerWorker | None = None
        # Connections are made in bind_device_manager()

    def _load_saved_device(self):
        try:
            if os.path.exists(self.SETTINGS_FILE):
                with open(self.SETTINGS_FILE, "r") as f:
                    addr = f.read().strip()
                if addr:
                    self.status.setText(f"Saved device available: {addr}")
                    # Do not auto-enable connect until user selects or loads
                    return
        except Exception:
            pass

    # UI handlers
    @QtCore.Slot()
    def on_scan(self):
        if self._scanner is None:
            self.status.setText("Scanner not ready")
            return
        self.btn_scan.setEnabled(False)
        self.btn_save_connect.setEnabled(False)
        self.list_devices.clear()
        self._pending_scan = True
        if self._qt_dev is not None and self._connected:
            try:
                self.status.setText("Disconnecting…")
                self._qt_dev.disconnect()
            except Exception:
                self._connected = False
                self._start_scan_now()
        else:
            self._start_scan_now()

    @QtCore.Slot()
    def on_load_saved(self):
        if os.path.exists(self.SETTINGS_FILE):
            try:
                with open(self.SETTINGS_FILE, "r") as f:
                    addr = f.read().strip()
                if addr:
                    self.selected_address = addr
                    self.status.setText(f"Connecting to saved device: {addr}")
                    # Disable Load and Save & Connect buttons during connection
                    # Keep Start Scan enabled so user can disconnect and start new scan
                    self.btn_load.setEnabled(False)
                    self.btn_save_connect.setEnabled(False)
                    # Auto-connect to saved device
                    self.connectRequested.emit(addr)
                    return
            except Exception:
                pass
        self.status.setText("No saved device found")

    @QtCore.Slot()
    def on_selected(self):
        items = self.list_devices.selectedItems()
        if items:
            item = items[0]
            self.selected_name = item.data(QtCore.Qt.UserRole)[0]
            self.selected_address = item.data(QtCore.Qt.UserRole)[1]
            self.btn_save_connect.setEnabled(True)
        else:
            self.selected_name = None
            self.selected_address = None
            self.btn_save_connect.setEnabled(False)

    @QtCore.Slot()
    def on_save_and_connect(self):
        if not self.selected_address:
            self.status.setText("Select a device first")
            return
        # Save selection
        try:
            os.makedirs(os.path.dirname(self.SETTINGS_FILE), exist_ok=True)
            with open(self.SETTINGS_FILE, "w") as f:
                f.write(self.selected_address)
        except Exception as ex:
            self.status.setText(f"Save failed: {ex}")
            return

        self.status.setText(f"Connecting to: {self.selected_name or ''} {self.selected_address}")
        self.btn_save_connect.setEnabled(False)
        # Emit request for Qt device manager to handle connection
        self.connectRequested.emit(self.selected_address)

    @QtCore.Slot()
    def on_calibrate_torque(self):
        if self._qt_dev is None:
            self.status.setText("Device manager not ready")
            return
        try:
            # Send torque calibration command
            self._qt_dev.calibrateTorque()

            # Keep Start Trial disabled for a short delay to allow calibration to settle
            self.btn_start_trial.setEnabled(False)

            self.status.setText("Torque calibration sent. Start Trial will be enabled in 3 seconds...")

            # Enable Start Trial after 3 seconds (only if still connected)
            def _enable_start_trial_if_connected():
                if self._connected and self._qt_dev is not None:
                    self.btn_start_trial.setEnabled(True)

            QtCore.QTimer.singleShot(1500, _enable_start_trial_if_connected)
        except Exception as ex:
            self.status.setText(f"Torque calibration failed: {ex}")

    @QtCore.Slot()
    def on_overwrite_boot_config(self):
        if self._qt_dev is None or not self._connected:
            self.status.setText("Connect to a device first")
            return

        dialog = OverwriteBootConfigDialog(self._qt_dev, self)
        self._active_overwrite_dialog = dialog
        try:
            dialog.exec()
        finally:
            self._active_overwrite_dialog = None

    @QtCore.Slot(list)
    def on_config_chunk_received(self, values):
        if self._active_overwrite_dialog is None:
            return
        self._active_overwrite_dialog.on_config_chunk_received(values)

    @QtCore.Slot(list)
    def on_overwrite_key_ack_received(self, values):
        if self._active_overwrite_dialog is None:
            return
        self._active_overwrite_dialog.on_overwrite_key_ack_received(values)

    # Called by MainWindow after it creates QtExoDeviceManager
    def bind_device_manager(self, qt_dev: QtExoDeviceManager):
        self._qt_dev = qt_dev
        self._scanner = DeviceScannerWorker(qt_dev)
        self._scanner.resultsReady.connect(self._on_scan_results)
        self._scanner.error.connect(self._on_error)
        qt_dev.connected.connect(self._on_device_connected)
        qt_dev.disconnected.connect(self._on_device_disconnected)

    # Worker callbacks
    @QtCore.Slot(list)
    def _on_scan_results(self, results: List[Tuple[str, str]]):
        self._pending_scan = False
        self.btn_scan.setEnabled(True)
        self.list_devices.clear()
        
        # Always re-enable Load button if saved device exists
        if os.path.exists(self.SETTINGS_FILE):
            try:
                with open(self.SETTINGS_FILE, "r") as f:
                    if f.read().strip():
                        self.btn_load.setEnabled(True)
            except Exception:
                pass
        
        if not results:
            self.status.setText("No devices found")
            return
        self.status.setText("Scan complete")
        for name, addr in results:
            item = QtWidgets.QListWidgetItem(f"{name} - {addr}")
            item.setData(QtCore.Qt.UserRole, (name, addr))
            self.list_devices.addItem(item)

    @QtCore.Slot(str, str)
    def _on_device_connected(self, name: str, address: str):
        self._connected = True
        # Keep Save & Connect disabled after connection
        self.btn_save_connect.setEnabled(False)
        self.btn_overwrite_boot.setEnabled(True)

    @QtCore.Slot()
    def _on_device_disconnected(self):
        self._connected = False
        self.btn_overwrite_boot.setEnabled(False)
        if self._pending_scan:
            self._start_scan_now()

    def _start_scan_now(self):
        self.status.setText("Scanning…")
        if self._scanner is None:
            self.status.setText("Scanner not ready")
            self.btn_scan.setEnabled(True)
            self._pending_scan = False
            return
        self._scanner.scan_once()

    @QtCore.Slot(bool, str)
    def _on_connected(self, ok: bool, msg: str):
        self.btn_save_connect.setEnabled(True)
        if ok:
            self.status.setText(f"Connected: {self.selected_name or ''} {self.selected_address}")
            self.btn_start_trial.setEnabled(True)
        else:
            self.status.setText(f"Connection failed: {msg}")

    @QtCore.Slot(str)
    def _on_error(self, message: str):
        self.status.setText(message)

    @QtCore.Slot(str)
    def _on_connect(self, message: str):
        self.status.setText(message)

def main():
    app = QtWidgets.QApplication(sys.argv)
    w = ScanWindowQt()
    w.show()
    sys.exit(app.exec())


if __name__ == "__main__":
    main()


