"Sample code for move absolute."
from ctypes import (
    c_short,
    c_int,
    c_char_p,
)
from time import sleep

from thorlabs_kinesis import kcube_stepper_motor as ksm


if __name__ == "__main__":
    serial_no = c_char_p(bytes("26002265", "utf-8"))
    milliseconds = c_int(100)

    if ksm.TLI_BuildDeviceList() == 0:
        err = ksm.SCC_Open(serial_no)
        if err == 0:
            print("Starting polling ", ksm.SCC_StartPolling(serial_no, milliseconds))
            print("Clearing message queue ", ksm.SCC_ClearMessageQueue(serial_no))
            sleep(0.2)

            move_to = 1000000
            print("Setting Absolute Position ", ksm.SCC_SetMoveAbsolutePosition(serial_no, c_int(move_to)))
            sleep(0.2)

            print(f"Moving to {move_to}", ksm.SCC_MoveAbsolute(serial_no))
            sleep(0.2)
            pos = int(ksm.SCC_GetPosition(serial_no))
            sleep(0.2)
            print(f"Current pos: {pos}")
            while not pos == move_to:
                sleep(0.2)
                pos = int(ksm.SCC_GetPosition(serial_no))
                print(f"Current pos: {pos}")

            print("Stopping polling ", ksm.SCC_StopPolling(serial_no))
            print("Closing connection ", ksm.SCC_Close(serial_no))
        else:
            print(f"Can't open. Error: {err}")
