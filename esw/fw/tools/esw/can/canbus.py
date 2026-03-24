import queue
import struct
from typing import Any, Callable, Tuple, cast

import can
from cantools.database import Database

from esw import esw_logger


def float2bits(value: float):
    if not isinstance(value, float):
        raise TypeError
    return struct.unpack("<I", struct.pack("<f", value))[0]


class CANBus:
    _MJBOTS_CAN_PREFIX: int = 0x0000
    _CAN_DEST_ID_MASK: int = 0x00FF
    _CAN_SRC_ID_MASK: int = 0xFF00
    _CAN_DEST_ID_OFFSET: int = 0
    _CAN_SRC_ID_OFFSET: int = 8

    _dbc: Database
    _channel: str
    _interface: str = "socketcan"
    _can_fd: bool = True
    _can_brs: bool = True
    _bus: can.BusABC | None
    _notifier: can.Notifier | None
    _rx_queue: "queue.Queue[Tuple[str, dict[str, Any], int, int]]"
    _on_msg: Callable

    def __init__(self, dbc: Database, channel: str, on_recv: Callable = lambda arg: None):
        self._dbc = dbc
        self._channel = channel
        self._bus = None
        self._rx_queue = queue.Queue()
        self._on_msg = on_recv

    def __enter__(self):
        if self._bus is None:
            esw_logger.info("Opening CAN Bus")
            self._bus = can.interface.Bus(channel=self._channel, interface=self._interface, fd=self._can_fd)
            esw_logger.info("Starting CAN Notifier")
            self._notifier = can.Notifier(self._bus, [self._on_message])
        return self

    def __exit__(self, exc_type, exc_value, traceback):
        if self._notifier is not None:
            esw_logger.info("Stopping CAN Notifier")
            self._notifier.stop()
            self._notifier = None

        if self._bus is not None:
            self._bus.shutdown()
            self._bus = None
            esw_logger.info("Closed CAN Bus")

        if exc_type:
            esw_logger.error(f"{exc_type.__class__.__name__}: {exc_value}")
            return False
        return False

    def get_dbc(self) -> Database:
        return self._dbc

    def _on_message(self, msg: can.Message) -> None:
        base_id = msg.arbitration_id & ~(self._CAN_SRC_ID_MASK | self._CAN_DEST_ID_MASK)
        src_id = (msg.arbitration_id & self._CAN_SRC_ID_MASK) >> self._CAN_SRC_ID_OFFSET
        dest_id = (msg.arbitration_id & self._CAN_DEST_ID_MASK) >> self._CAN_DEST_ID_OFFSET
        try:
            dbc_msg = self._dbc.get_message_by_frame_id(base_id)
            raw_data = bytes(msg.data) if msg.data is not None else b""
            decoded_signals = cast(dict[str, Any], dbc_msg.decode(raw_data))
            self._rx_queue.put((dbc_msg.name, decoded_signals, src_id, dest_id))
            esw_logger.debug(f"CAN RECV {dbc_msg.name} (src: {src_id}, dest: {dest_id}): {decoded_signals}")
            self._on_msg((dbc_msg.name, decoded_signals, src_id, dest_id))

        except Exception as e:
            if base_id == self._MJBOTS_CAN_PREFIX:
                esw_logger.debug("CAN RECV moteus message")
                # TODO(eric) implement some handler for this?
            else:
                esw_logger.error(f"CAN ERROR: decode error on ID {hex(msg.arbitration_id)}: {e}")

    def send(self, message_name: str, signals: dict[str, Any], src_id: int = 0, dest_id: int = 0) -> None:
        if self._bus is None:
            esw_logger.error("CAN ERROR: CAN Bus Not Active")
            return

        processed_signals = signals.copy()

        try:
            dbc_msg = self._dbc.get_message_by_name(message_name)
            data = dbc_msg.encode(processed_signals)
            msg = can.Message(
                arbitration_id=dbc_msg.frame_id + (src_id << 8) + dest_id,
                data=data,
                is_extended_id=dbc_msg.is_extended_frame,
                is_fd=self._can_fd,
                bitrate_switch=self._can_brs,
                check=True,
            )
            self._bus.send(msg)
            esw_logger.info(f"CAN SEND {message_name}: {signals}")

        except KeyError:
            esw_logger.error(f"CAN ERROR: Message '{message_name}' not in DBC")
        except can.CanError as e:
            esw_logger.error(f"CAN ERROR: failed to send message: {e}")
        except Exception as e:
            esw_logger.error(f"CAN ERROR: exception when sending message {message_name}: {e}")

    def recv(self, block: bool = True, timeout: float | None = None) -> Tuple[str, dict[str, Any], int, int] | None:
        try:
            return self._rx_queue.get(block=block, timeout=timeout)
        except queue.Empty:
            return None
