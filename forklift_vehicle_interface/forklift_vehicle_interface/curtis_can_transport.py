from __future__ import annotations

import select
import socket
import struct
from dataclasses import dataclass
from typing import Iterable, List, Optional


CAN_FRAME_SIZE = 16
CAN_MAX_DLEN = 8
CAN_EFF_MASK = 0x1FFFFFFF
_CAN_FRAME_STRUCT = struct.Struct('=IB3x8s')


@dataclass(frozen=True)
class CanFrame:
    can_id: int
    data: List[int]


class SocketCanTransport:
    """Minimal SocketCAN transport for standard 8-byte Curtis frames."""

    def __init__(self, interface: str) -> None:
        if not interface:
            raise ValueError('SocketCAN interface name must not be empty')
        self._interface = interface
        self._socket: Optional[socket.socket] = None

    @property
    def interface(self) -> str:
        return self._interface

    def open(self) -> None:
        if self._socket is not None:
            return
        sock = socket.socket(socket.AF_CAN, socket.SOCK_RAW, socket.CAN_RAW)
        sock.bind((self._interface,))
        sock.setblocking(False)
        self._socket = sock

    def close(self) -> None:
        sock = self._socket
        self._socket = None
        if sock is not None:
            sock.close()

    def send(self, can_id: int, data: Iterable[int]) -> None:
        sock = self._require_socket()
        frame_data = _normalize_data(data)
        payload = _CAN_FRAME_STRUCT.pack(can_id & CAN_EFF_MASK, len(frame_data), bytes(frame_data))
        sock.send(payload)

    def receive(self, timeout_sec: float = 0.0) -> Optional[CanFrame]:
        sock = self._require_socket()
        readable, _, _ = select.select([sock], [], [], max(0.0, timeout_sec))
        if not readable:
            return None
        frame = sock.recv(CAN_FRAME_SIZE)
        if len(frame) != CAN_FRAME_SIZE:
            return None
        can_id, dlc, data = _CAN_FRAME_STRUCT.unpack(frame)
        dlc = min(int(dlc), CAN_MAX_DLEN)
        return CanFrame(can_id=can_id & CAN_EFF_MASK, data=list(data[:dlc]))

    def receive_available(self, max_frames: int = 16) -> List[CanFrame]:
        frames: List[CanFrame] = []
        for _ in range(max(0, max_frames)):
            frame = self.receive(timeout_sec=0.0)
            if frame is None:
                break
            frames.append(frame)
        return frames

    def _require_socket(self) -> socket.socket:
        if self._socket is None:
            raise RuntimeError('SocketCAN transport is not open')
        return self._socket


def _normalize_data(data: Iterable[int]) -> List[int]:
    frame = [int(byte) & 0xFF for byte in data]
    if len(frame) != CAN_MAX_DLEN:
        raise ValueError(f'CAN frame payload must be {CAN_MAX_DLEN} bytes, got {len(frame)}')
    return frame
