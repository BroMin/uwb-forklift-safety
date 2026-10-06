import serial, os, re, time

base = '/dev/serial/by-id/'
port = [os.path.join(base, e) for e in os.listdir(base)
        if '760197764' in e and 'if00' in e][0]

ser = serial.Serial(port, 115200, timeout=1)
print(f"접속: {port}")
print("10초간 TSYNC 값 수집...\n")

buf = ''
found = []
deadline = time.time() + 10

while time.time() < deadline:
    chunk = ser.read(ser.in_waiting or 1).decode('utf-8', errors='ignore')
    if not chunk:
        continue
    buf += chunk
    lines = buf.split('\n')
    buf = lines[-1]
    for line in lines[:-1]:
        m = re.search(r'TSYNC:\s*(0x[\dA-Fa-f]+)\s+ID:\s*([\dA-Fa-f]+)', line)
        if m:
            found.append(m.group(1))
            print(f"  {m.group(1)}")

ser.close()
print(f"\n총 {len(found)}개 수집")
if len(found) >= 2:
    vals = [int(v, 16) for v in found]
    all_same = all(v == vals[0] for v in vals)
    print("고정값 (문제)" if all_same else "값 변화함 (정상)")
    if not all_same:
        print(f"첫 값: {found[0]}  마지막 값: {found[-1]}")
