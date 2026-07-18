import socket, json, time, sys

SOCK = "/tmp/klippy.sock"
# wait for the socket to appear (klippy still starting)
for _ in range(60):
    try:
        s = socket.socket(socket.AF_UNIX, socket.SOCK_STREAM); s.connect(SOCK); break
    except OSError:
        time.sleep(0.5)
else:
    print("CLIENT: could not connect to klippy socket"); sys.exit(1)

def send(obj):
    s.sendall(json.dumps(obj).encode() + b"\x03")

# subscribe to gcode output, then dump each TMC
send({"id":1,"method":"gcode/subscribe_output","params":{"response_template":{"k":"resp"}}})
time.sleep(0.5)
for i,axis in enumerate(("stepper_x","stepper_y","stepper_z"), start=2):
    send({"id":i,"method":"gcode/script","params":{"script":f"DUMP_TMC STEPPER={axis}"}})

buf=b""; end=time.time()+8
s.settimeout(1.0)
lines=[]
while time.time()<end:
    try: data=s.recv(65536)
    except socket.timeout: continue
    if not data: break
    buf+=data
    while b"\x03" in buf:
        msg,buf=buf.split(b"\x03",1)
        try: o=json.loads(msg)
        except Exception: continue
        r=o.get("params",{}).get("response")
        if r: lines.append(r)
        if "error" in o: lines.append("ERROR: "+json.dumps(o["error"]))
print("==== DUMP_TMC responses ====")
for l in lines: print(l)
print(f"==== {len(lines)} response lines ====")
