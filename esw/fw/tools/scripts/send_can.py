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
    TARGET = -0.7#0.05 # m
    INC = 0.10
    CAN_ID = 53
    SRC_ID = 10

    with CANBus(get_dbc(dbc_name="CANBus1"), "can0", on_recv=on_msg_recv) as bus:
        sleep(1)
        bus.send("BMCModeCmd", {"mode": 5, "enable": 1}, src_id=SRC_ID, dest_id=CAN_ID)
        while True:
            for _ in range(NUM_LOOPS):
                bus.send("BMCTargetCmd", {"target": TARGET, "target_valid": 1}, src_id=SRC_ID, dest_id=CAN_ID)
                sleep(LOOP_DELAY)

        sleep(5)
        # set mode to position
        bus.send("BMCModeCmd", {"mode": 6, "enable": 1}, src_id=SRC_ID, dest_id=CAN_ID)
        while True:
            for _ in range(NUM_LOOPS):
                bus.send("BMCTargetCmd", {"target": TARGET, "target_valid": 1}, src_id=SRC_ID, dest_id=CAN_ID)
                sleep(LOOP_DELAY)






            # TARGET += INC
            # if abs(TARGET - 1.0) < 0.01 or abs(TARGET + 1.0) < 0.01:
            #     INC *= -1

        sleep(50)
