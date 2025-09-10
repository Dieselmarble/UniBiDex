import time
from threading import Event, Lock, Thread
from typing import Protocol, Sequence, Tuple
import numpy as np
from dynamixel_sdk.group_sync_read import GroupSyncRead
from dynamixel_sdk.group_sync_write import GroupSyncWrite
from dynamixel_sdk.packet_handler import PacketHandler
from dynamixel_sdk.port_handler import PortHandler
from dynamixel_sdk.robotis_def import (
    COMM_SUCCESS,
    DXL_HIBYTE,
    DXL_HIWORD,
    DXL_LOBYTE,
    DXL_LOWORD,
)

# Constants for position
ADDR_TORQUE_ENABLE    = 64
ADDR_GOAL_POSITION    = 116
LEN_GOAL_POSITION     = 4
ADDR_PRESENT_POSITION = 132
LEN_PRESENT_POSITION  = 4

# Constants for velocity
ADDR_PRESENT_VELOCITY = 128
LEN_PRESENT_VELOCITY  = 4

# Constants for current control (Protocol 2.0)
ADDR_GOAL_CURRENT     = 102
LEN_GOAL_CURRENT      = 2
ADDR_OPERATING_MODE   = 11

TORQUE_ENABLE  = 1
TORQUE_DISABLE = 0


class DynamixelDriverProtocol(Protocol):
    def set_joints(self, joint_angles: Sequence[float]): ...
    def torque_enabled(self) -> bool: ...
    def set_torque_mode(self, enable: bool): ...
    def get_joints(self) -> np.ndarray: ...
    def get_joint_speeds(self) -> np.ndarray: ...
    def get_positions_and_velocities(self) -> Tuple[np.ndarray, np.ndarray]: ...
    def set_current(self, currents: Sequence[int]): ...
    def close(self): ...


class DynamixelDriver(DynamixelDriverProtocol):
    def __init__(
        self, ids: Sequence[int], port: str = "/dev/ttyUSB0", baudrate: int = 57600
    ):
        self._ids = ids
        self._joint_angles = None
        self._joint_velocities = None
        self._lock = Lock()

        # Handlers
        self._portHandler   = PortHandler(port)
        self._packetHandler = PacketHandler(2.0)

        # Sync read: position and velocity
        self._groupSyncRead    = GroupSyncRead(
            self._portHandler,
            self._packetHandler,
            ADDR_PRESENT_POSITION,
            LEN_PRESENT_POSITION,
        )
        self._groupSyncReadVel = GroupSyncRead(
            self._portHandler,
            self._packetHandler,
            ADDR_PRESENT_VELOCITY,
            LEN_PRESENT_VELOCITY,
        )
        # Sync write: position and current
        self._groupSyncWritePos = GroupSyncWrite(
            self._portHandler,
            self._packetHandler,
            ADDR_GOAL_POSITION,
            LEN_GOAL_POSITION,
        )
        self._groupSyncWriteCur = GroupSyncWrite(
            self._portHandler,
            self._packetHandler,
            ADDR_GOAL_CURRENT,
            LEN_GOAL_CURRENT,
        )

        # Open port and set baudrate
        if not self._portHandler.openPort():
            raise RuntimeError("Failed to open port")
        if not self._portHandler.setBaudRate(baudrate):
            raise RuntimeError(f"Failed to set baudrate {baudrate}")

        # Register IDs for reading
        for dxl_id in self._ids:
            self._groupSyncRead.addParam(dxl_id)
            self._groupSyncReadVel.addParam(dxl_id)

        # Start reading thread
        self._stop_thread = Event()
        self._reading_thread = Thread(target=self._read_joint_angles)
        self._reading_thread.daemon = True
        self._reading_thread.start()

        # Torque off initially
        self._torque_enabled = False

    def set_joints(self, joint_angles: Sequence[float]):
        if not self._torque_enabled:
            raise RuntimeError("Torque must be enabled before setting joints")
        if len(joint_angles) != len(self._ids):
            raise ValueError("Length mismatch between joint_angles and IDs")
        with self._lock:
            # Add params for batch write
            for dxl_id, angle in zip(self._ids, joint_angles):
                pos_val = int(angle * 2048 / np.pi)
                params = [
                    DXL_LOBYTE(DXL_LOWORD(pos_val)),
                    DXL_HIBYTE(DXL_LOWORD(pos_val)),
                    DXL_LOBYTE(DXL_HIWORD(pos_val)),
                    DXL_HIBYTE(DXL_HIWORD(pos_val)),
                ]
                ok = self._groupSyncWritePos.addParam(dxl_id, params)
                if not ok:
                    raise RuntimeError(f"Failed to add position param for ID {dxl_id}")
            # Send packet
            comm_result = self._groupSyncWritePos.txPacket()
            if comm_result != COMM_SUCCESS:
                # Detailed debug
                print(f"[ERROR] SyncWritePos failed: code={comm_result}")
                # Fallback to individual writes
                for dxl_id, angle in zip(self._ids, joint_angles):
                    pos_val = int(angle * 2048 / np.pi)
                    res, err = self._packetHandler.write4ByteTxRx(
                        self._portHandler, dxl_id,
                        ADDR_GOAL_POSITION,
                        pos_val
                    )
                    if res != COMM_SUCCESS or err != 0:
                        raise RuntimeError(f"Fallback write failed for ID {dxl_id}: res={res}, err={err}")
            self._groupSyncWritePos.clearParam()

    def set_current(self, currents: Sequence[int]):
        if not self._torque_enabled:
            raise RuntimeError("Torque must be enabled before setting current")
        if len(currents) != len(self._ids):
            raise ValueError("Length mismatch between currents and IDs")
        with self._lock:
            for dxl_id, cur in zip(self._ids, currents):
                cur_val = int(cur)
                params = [DXL_LOBYTE(cur_val), DXL_HIBYTE(cur_val)]
                ok = self._groupSyncWriteCur.addParam(dxl_id, params)
                if not ok:
                    raise RuntimeError(f"Failed to add current param for ID {dxl_id}")
            result = self._groupSyncWriteCur.txPacket()
            if result != COMM_SUCCESS:
                raise RuntimeError(f"SyncWrite current failed: code={result}")
            self._groupSyncWriteCur.clearParam()

    def torque_enabled(self) -> bool:
        return self._torque_enabled

    def set_torque_mode(self, enable: bool):
        value = TORQUE_ENABLE if enable else TORQUE_DISABLE
        with self._lock:
            for dxl_id in self._ids:
                res, err = self._packetHandler.write1ByteTxRx(
                    self._portHandler, dxl_id, ADDR_TORQUE_ENABLE, value
                )
                if res != COMM_SUCCESS or err != 0:
                    raise RuntimeError(f"Failed to set torque for ID {dxl_id}")
        self._torque_enabled = enable

    def _read_joint_angles(self):
        while not self._stop_thread.is_set():
            time.sleep(0.001)
            with self._lock:
                if self._groupSyncRead.txRxPacket() != COMM_SUCCESS:
                    continue
                if self._groupSyncReadVel.txRxPacket() != COMM_SUCCESS:
                    continue
                pos_list, vel_list = [], []
                for dxl_id in self._ids:
                    if self._groupSyncRead.isAvailable(
                        dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION
                    ):
                        raw_pos = self._groupSyncRead.getData(
                            dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION
                        )
                        pos_list.append(np.int32(np.uint32(raw_pos)))
                    if self._groupSyncReadVel.isAvailable(
                        dxl_id, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY
                    ):
                        raw_vel = self._groupSyncReadVel.getData(
                            dxl_id, ADDR_PRESENT_VELOCITY, LEN_PRESENT_VELOCITY
                        )
                        vel_list.append(np.int32(np.uint32(raw_vel)))
                if pos_list:
                    self._joint_angles = np.array(pos_list)
                if vel_list:
                    self._joint_velocities = np.array(vel_list)

    def get_joints(self) -> np.ndarray:
        while self._joint_angles is None:
            time.sleep(0.01)
        return (self._joint_angles.copy() / 2048.0) * np.pi

    def get_joint_speeds(self) -> np.ndarray:
        while self._joint_velocities is None:
            time.sleep(0.01)
        return self._joint_velocities.copy()

    def get_positions_and_velocities(self) -> Tuple[np.ndarray, np.ndarray]:
        return self.get_joints(), self.get_joint_speeds()

    def close(self):
        self._stop_thread.set()
        self._reading_thread.join()
        self._portHandler.closePort()


def main():
    ids = [1]
    try:
        driver = DynamixelDriver(ids)
    except FileNotFoundError:
        driver = DynamixelDriver(ids, port="/dev/cu.usbserial-FT7WBMUB")

    # Switch to Mode 5: Current-based Position Control
    for dxl_id in driver._ids:
        driver._packetHandler.write1ByteTxRx(
            driver._portHandler, dxl_id, ADDR_OPERATING_MODE, 5
        )

    driver.set_torque_mode(True)

    try:
        while True:
            pos, vel = driver.get_positions_and_velocities()
            driver.set_joints(pos)
            # example torque limit
            driver.set_current([200] * len(ids))
            print(f"Pos={pos}, Vel={vel}")
            time.sleep(0.01)
    except KeyboardInterrupt:
        pass
    finally:
        driver.set_torque_mode(False)
        driver.close()

if __name__ == "__main__":
    main()
