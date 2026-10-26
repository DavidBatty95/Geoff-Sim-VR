"""Generate a local development certificate; never overwrite existing keys."""
import ipaddress
import os
from pathlib import Path
import subprocess
import sys

def main():
    address = str(ipaddress.ip_address(sys.argv[1]))
    directory = Path(__file__).resolve().parents[1] / 'certs'
    directory.mkdir(exist_ok=True, mode=0o700)
    cert, key = directory / 'cert.pem', directory / 'key.pem'
    if cert.exists() or key.exists():
        raise SystemExit('Certificates already exist. Keep them, or explicitly move both files before regenerating.')
    os.umask(0o077)
    subprocess.run(['openssl', 'req', '-x509', '-nodes', '-days', '365', '-newkey', 'rsa:2048',
                    '-keyout', str(key), '-out', str(cert), '-subj', '/CN=Dougie-VR',
                    '-addext', f'subjectAltName=DNS:localhost,IP:127.0.0.1,IP:{address}'], check=True)
    print(f'Quest URL: https://{address}:8012/?ws=wss://{address}:8012')
    print('Development certificate: the headset must trust it before WebXR is available.')

if __name__ == '__main__':
    main()
