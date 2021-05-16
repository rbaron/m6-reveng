import argparse
import serial
import time

BAUD = 115200 * 2

SLEEP_AFTER_RESET_IN_S = 0.02
SLEEP_BETWEEN_READ_AND_WRITE_IN_S = 0.02

RAM_SIZE = 0x010000
FLASH_SIZE = 0x7d000

serial_port = None


def hexdump(bytearr):
    return ' '.join(f'{b:02x}' for b in bytearr)


def make_read_request(addr, n_bytes):
    return [0x5a, addr >> 8, addr & 0xff, 0x80] + [0xff] * (n_bytes + 1)


def make_write_request(addr, data):
    return [0x5a, addr >> 8, addr & 0xff, 0x00] + data + [0xff]


def request_payload_size(request):
    "Discounts the leading 4 bytes and trailing 1 byte from read and write requests."
    return len(request) - 5


def validate_raw_response(request, raw_response):
    return all([
        raw_response[0] == 0x02,
        raw_response[1] == request[0],
        raw_response[2] == 0x00,
        raw_response[3] == request[1],
        raw_response[4] == 0x00,
        raw_response[5] == request[2],
        raw_response[6] == 0x00,
        raw_response[7] == request[3],
        raw_response[-2] == 0x02,
        raw_response[-1] == 0xff,
    ])


def parse_response(raw_response, n_bytes):
    "Extracts the data bytes from the raw response."
    return [
        raw_response[(i+4)*2 + 1]
        for i in range(n_bytes)
    ]


def write_and_read(data):
    serial_port.write(data)
    time.sleep(SLEEP_BETWEEN_READ_AND_WRITE_IN_S)
    return serial_port.read_all()


def write_and_read_cmd(cmd, data=None):
    cmd_bytes = bytearray([0x55, cmd] if data is None else [0x55, cmd] + data)
    return write_and_read(cmd_bytes)


def write_and_read_data(request):
    payload_size = request_payload_size(request)
    data_bytes = bytearray(request)
    raw_response = write_and_read(data_bytes)
    if not validate_raw_response(request, raw_response):
        raise Exception(
            f"Invalid request/raw response pair:\n\t{hexdump(request)}\n\t{hexdump(raw_response)}")
    return parse_response(raw_response, payload_size)


def get_soc_id():
    res = write_and_read_data(make_read_request(0x007e, 2))
    return res[1] << 8 | res[0]


def set_speed(speed):
    """The SWS speed is set in the 0x00b2 register by means of
        specifying the number of clocks per bit."""
    return write_and_read_data(make_write_request(0x00b0, [0x00, 0x80, speed, 0x00]))


def find_suitable_sws_speed():
    for speed in range(2, 0x7f):
        set_speed(speed)
        # Try to make a read request. If we get a valid response, assume we've
        # found a suitable SWS speed.
        try:
            get_soc_id()
        except Exception:
            continue
        else:
            print(f'Found and set suitable SWS speed: {speed}')
            return speed
    raise RuntimeError("Unable to find a suitable SPI speed")


def send_flash_write_enable():
    # CNS low.
    write_and_read_data(make_write_request(0x0d, [0x00]))
    # Write enable.
    write_and_read_data(make_write_request(0x0c, [0x06]))
    # CNS high.
    write_and_read_data(make_write_request(0x0d, [0x01]))


def send_cpu_stop():
    return write_and_read_data(make_write_request(0x0602, [0x05]))


def send_csn_high():
    return write_and_read_data(make_write_request(0x000d, [0x01]))


def send_csn_low():
    return write_and_read_data(make_write_request(0x000d, [0x00]))


def dump_ram():
    contents = []
    CHUNK_SIZE = 32
    for addr in range(0x00, RAM_SIZE, CHUNK_SIZE):
        # Report progress.
        if addr & 0xff == 0:
            print(f'0x{addr:04x} {100 * addr / RAM_SIZE:05.2f}%')
        contents.extend(write_and_read_data(
            make_read_request(addr, CHUNK_SIZE)))
    return contents


def read_flash(addr, chunk_size):
    contents = []
    send_flash_write_enable()
    # CNS low.
    write_and_read_data(make_write_request(0x0d, [0x00]))
    # Read command.
    write_and_read_data(make_write_request(0x0c, [0x03]))
    write_and_read_data(make_write_request(0x0c, [(addr >> 16) & 0xff]))
    write_and_read_data(make_write_request(0x0c, [(addr >> 8) & 0xff]))
    write_and_read_data(make_write_request(0x0c, [addr & 0xff]))

    for i in range(chunk_size):
        write_and_read_data(make_write_request(0x0c, [0xff]))
        res = write_and_read_data(make_read_request(0x0c, 1))
        assert len(res) == 1
        contents.extend(res)

    # CNS high.
    write_and_read_data(make_write_request(0x0d, [0x01]))
    return contents


def dump_flash():
    contents = []
    print('CPU stop.')
    send_cpu_stop()
    print('CSN high.')
    send_csn_high()

    CHUNK_SIZE = 32
    for addr in range(0x00, FLASH_SIZE, CHUNK_SIZE):
        # Report progress.
        if addr & 0xff == 0:
            print(f'0x{addr:06x} {100 * addr / FLASH_SIZE:05.2f}%')
        # Retry the same address in case something goes wrong.
        while True:
            try:
                contents.extend(read_flash(addr, CHUNK_SIZE))
                break
            except Exception as e:
                print(f"Retrying 0x{addr:08x}... {e}")

    return contents


def write_to_file(filename, contents):
    print(f"Writing {len(contents)} bytes to {filename}")
    with open(filename, 'wb') as f:
        f.write(bytes(contents))


def init_soc(sws_speed=None):
    # Set RST to low - turns of the SoC.
    write_and_read_cmd(0x00)
    # Set RST high - starts to turn on the SoC.
    write_and_read_cmd(0x01)
    # Give some time for the reset capacitor to charge and turn the chip on.
    time.sleep(SLEEP_AFTER_RESET_IN_S)

    # Send an "activate" command. The STM32 will receive this command and put the Telink
    # in a suitable state. The STM32 will stop the Telink CPU by writing the value
    # 0x05 to Telink's 0x0602 register. It will also set a default SWS speed, but we
    # will override it later when we find a suitable SWS speed.
    write_and_read_cmd(0x02, [0x00, 0x00])

    if sws_speed is not None:
        set_speed(sws_speed)
    else:
        find_suitable_sws_speed()


def dump_flash_main(args):
    init_soc(args.sws_speed)
    print(f'Dumping flash to {args.filename}...')
    # Speed things up a little bit.
    global SLEEP_BETWEEN_READ_AND_WRITE_IN_S
    SLEEP_BETWEEN_READ_AND_WRITE_IN_S = 0.0005
    write_to_file(args.filename, dump_flash())


def dump_ram_main(args):
    init_soc(args.sws_speed)
    print(f'Dumping ram to {args.filename}...')
    # Speed things up a little bit.
    global SLEEP_BETWEEN_READ_AND_WRITE_IN_S
    SLEEP_BETWEEN_READ_AND_WRITE_IN_S = 0.005
    write_to_file(args.filename, dump_ram())


def get_soc_id_main(args):
    init_soc(args.sws_speed)
    print(f'SOC ID: 0x{get_soc_id():04x}')


def main():
    args_parser = argparse.ArgumentParser(description='TLSR')
    args_parser.add_argument('--serial-port', type=str, required=True,
                             help="Serial port to use - this should be the USB CDC port that is connected to the STM32 (e.g.: /dev/cu.usbmodem6D8E448E55511.")
    args_parser.add_argument(
        '--sws-speed', type=int, help="SWS speed in the range [0x02, 0x7f]. If not provided, the script will try to find a suitable SWS speed automatically.")
    subparsers = args_parser.add_subparsers(dest="cmd", required=True)

    dump_flash_parser = subparsers.add_parser('dump_flash')
    dump_flash_parser.set_defaults(func=dump_flash_main)
    dump_flash_parser.add_argument('filename', type=str)

    dump_ram_parser = subparsers.add_parser('dump_ram')
    dump_ram_parser.set_defaults(func=dump_ram_main)
    dump_ram_parser.add_argument('filename', type=str)

    get_soc_id_parser = subparsers.add_parser('get_soc_id')
    get_soc_id_parser.set_defaults(func=get_soc_id_main)

    args = args_parser.parse_args()

    # Initialize the serial port
    global serial_port
    serial_port = serial.Serial(
        args.serial_port, BAUD, serial.EIGHTBITS, serial.PARITY_NONE, serial.STOPBITS_ONE)

    args.func(args)


if __name__ == "__main__":
    main()
