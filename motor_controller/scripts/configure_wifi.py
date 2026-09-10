"""Write an ignored build-local station credential header without logging secrets."""

import argparse
from getpass import getpass
import json
import os
from pathlib import Path
import sys


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument('--ssid', required=True)
    parser.add_argument('--password-stdin', action='store_true')
    args = parser.parse_args()
    password = (
        sys.stdin.readline().rstrip('\r\n')
        if args.password_stdin else getpass('Wi-Fi password: ')
    )
    if not 1 <= len(args.ssid.encode()) <= 31:
        parser.error('SSID must contain 1..31 UTF-8 bytes')
    if not 8 <= len(password.encode()) <= 63:
        parser.error('WPA2 passphrase must contain 8..63 UTF-8 bytes')
    destination = Path(__file__).resolve().parents[1] / 'src/wifi_credentials.local.h'
    content = (
        '// Local station credentials; do not commit or share firmware images.\n'
        '#pragma once\n'
        f'#define CLEANY_STA_SSID {json.dumps(args.ssid, ensure_ascii=False)}\n'
        f'#define CLEANY_STA_PASSWORD {json.dumps(password, ensure_ascii=False)}\n'
    )
    descriptor = os.open(destination, os.O_WRONLY | os.O_CREAT | os.O_TRUNC, 0o600)
    with os.fdopen(descriptor, 'w') as output:
        os.fchmod(output.fileno(), 0o600)
        output.write(content)
    print('Station credentials saved in ignored local header')


if __name__ == '__main__':
    main()
