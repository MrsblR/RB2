import socket, struct
import numpy as np
from controller import Supervisor

HOST, PORT = "127.0.0.1", 5555

def send_state_recv_action(sock, state_np):
    # state_np: float32 shape (STATE_DIM,)
    payload = state_np.astype(np.float32).tobytes()
    sock.sendall(struct.pack("!I", len(payload)) + payload)
    # recibe 4 bytes int acción
    data = sock.recv(4)
    return struct.unpack("!I", data)[0]

robot = Supervisor()
dt = int(robot.getBasicTimeStep())

# ... inicializa motores, sensores, nodos igual que tu código ...

sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
sock.connect((HOST, PORT))

while robot.step(dt) != -1:
    # 1) sense() -> disc/raw, distancia -> state
    disc, raw = sense()
    rx, rz = get_robot_xz()
    tx, tz = get_target_xz()
    d  = distance(rx, rz, tx, tz)
    dn = min(1.0, d / 0.5)
    s = build_state(disc, dn)  # shape (5,)

    # 2) acción desde learner (CUDA)
    action = send_state_recv_action(sock, s)

    # 3) act(action)
    act(action, raw)

    # 4) step
    robot.step(dt)

    # 5) next state + reward + done (igual que tu lógica)
    disc2, raw2 = sense()
    rx2, rz2 = get_robot_xz()
    d2 = distance(rx2, rz2, tx, tz)
    dn2 = min(1.0, d2 / 0.5)
    s2 = build_state(disc2, dn2)

    r = compute_reward(disc2, d, d2)
    done = False
    # ... tu chequeo de contacto ...

    # 6) manda transición al learner (opcional en el mismo canal)
    # Puedes mandar un “tipo de mensaje”: (s,a,r,s2,done)
    # Para hacerlo corto aquí, lo dejo como siguiente mejora.

sock.close()
