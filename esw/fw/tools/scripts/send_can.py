from time import sleep

from esw import esw_logger
from esw.can.dbc import get_dbc
from esw.can.canbus import CANBus


def on_msg_recv(msg):
    msg_name, signals, src_id, dest_id = msg
    # if msg_name != "BMCMotorState":
    #     # pass
    #     esw_logger.info(f"CAN RECV {msg_name} (src: {hex(src_id)}, dest: {hex(dest_id)}): {signals}")
    # else:
    esw_logger.info(f"CAN RECV {msg_name} (src: {hex(src_id)}, dest: {hex(dest_id)}): {signals}")


if __name__ == "__main__":
    NUM_LOOPS = 50
    LOOP_DELAY = 0.05
    TARGET = 0.8  # -0.008  # 0.01  # m/s
    INC = 0.1
    CAN_ID = 54
    SRC_ID = 16

    with CANBus(get_dbc(dbc_name="MRoverCAN"), "can2", on_recv=on_msg_recv) as bus:
        # sleep(1000000)

        # if abs(TARGET) == 1.0:
        print("THROTTLE")
        sleep(2)
        i = 0
        bus.send("BMCModeCmd", {"mode": 5, "enable": 1}, src_id=SRC_ID, dest_id=CAN_ID)
        while True:
            for _ in range(NUM_LOOPS):
                bus.send("BMCTargetCmd", {"target": TARGET, "target_valid": 1}, src_id=SRC_ID, dest_id=CAN_ID)
                sleep(LOOP_DELAY)
                i += 1
                if i % 100 == 0:
                    TARGET *= -1

        # print("POSITION")
        # sleep(2)
        # # set mode to position
        # bus.send("BMCModeCmd", {"mode": 6, "enable": 1}, src_id=SRC_ID, dest_id=CAN_ID)
        # while True:
        #     for _ in range(NUM_LOOPS):
        #         bus.send("BMCTargetCmd", {"target": TARGET, "target_valid": 1}, src_id=SRC_ID, dest_id=CAN_ID)
        #         sleep(LOOP_DELAY)

        # print("VELOCITY")
        # sleep(2)
        # # set mode to velocity
        # i = 0
        # for _ in range(5):
        #     bus.send("BMCModeCmd", {"mode": 7, "enable": 1}, src_id=SRC_ID, dest_id=CAN_ID)
        # while True:
        #     for _ in range(NUM_LOOPS):
        #         bus.send("BMCTargetCmd", {"target": TARGET, "target_valid": 1}, src_id=SRC_ID, dest_id=CAN_ID)
        #         sleep(LOOP_DELAY)

        sleep(50)
