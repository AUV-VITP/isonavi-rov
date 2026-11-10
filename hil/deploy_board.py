"""Deploy the flight-computer code onto the LicheeRV Nano over SFTP.

Copies only what the board needs: the protocol, the ROV flight computer and its
links, and the simulation modules it imports (dynamics, pilot assist, safety
and what they build on). The board gets a self-contained /root/hil tree.
"""
import os
import posixpath

import paramiko

IP = "10.133.84.1"
REPO = os.path.expanduser("~/dev/isonavi-rov")  # runs under WSL
# When run from Windows python this path differs; allow override.
REPO = os.environ.get("REPO", REPO)

FILES = [
    ("hil/common/hil_protocol.py", "hil/common/hil_protocol.py"),
    ("hil/board/links.py", "hil/links.py"),
    ("hil/board/rov_computer.py", "hil/rov_computer.py"),
    ("simulation/vehicle/rov_budget.json", "hil/vehicle/rov_budget.json"),
    ("simulation/isonavi/__init__.py", "hil/isonavi/__init__.py"),
    ("simulation/isonavi/control.py", "hil/isonavi/control.py"),
    ("simulation/isonavi/dynamics.py", "hil/isonavi/dynamics.py"),
    ("simulation/isonavi/geometry.py", "hil/isonavi/geometry.py"),
    ("simulation/isonavi/optics.py", "hil/isonavi/optics.py"),
    ("simulation/isonavi/pilot.py", "hil/isonavi/pilot.py"),
    ("simulation/isonavi/pool.py", "hil/isonavi/pool.py"),
    ("simulation/isonavi/safety.py", "hil/isonavi/safety.py"),
    ("simulation/isonavi/sensors.py", "hil/isonavi/sensors.py"),
    ("simulation/isonavi/vision.py", "hil/isonavi/vision.py"),
]

c = paramiko.SSHClient()
c.set_missing_host_key_policy(paramiko.AutoAddPolicy())
c.connect(IP, username="root", password="root", timeout=10,
          allow_agent=False, look_for_keys=False)
sftp = c.open_sftp()


def mkdirs(remote_dir):
    parts = remote_dir.strip("/").split("/")
    cur = ""
    for p in parts:
        cur = cur + "/" + p
        try:
            sftp.stat(cur)
        except IOError:
            sftp.mkdir(cur)


ROOT = "/root"
for local_rel, remote_rel in FILES:
    local = os.path.join(REPO, *local_rel.split("/"))
    remote = posixpath.join(ROOT, remote_rel)
    mkdirs(posixpath.dirname(remote))
    sftp.put(local, remote)
    size = os.path.getsize(local)
    print(f"  {remote_rel}  ({size} bytes)")

# The board needs common/ importable as a package too.
try:
    sftp.stat("/root/hil/common/__init__.py")
except IOError:
    with sftp.open("/root/hil/common/__init__.py", "w") as f:
        f.write("")

sftp.close()

# Quick import smoke test on the board.
_, out, err = c.exec_command(
    "cd /root/hil && python3 -c '"
    "import sys; sys.path.insert(0,\".\"); sys.path.insert(0,\"common\"); "
    "import hil_protocol; from isonavi.pilot import AssistController; "
    "from rov_computer import RovComputer; "
    "print(\"board imports OK\")'", timeout=30)
print("  " + (out.read().decode() + err.read().decode()).strip())
c.close()
