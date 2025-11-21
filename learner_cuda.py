import socket, struct, random
import numpy as np
import torch
import torch.nn as nn
import torch.optim as optim
from collections import deque

# =========================
# Config
# =========================
HOST, PORT = "127.0.0.1", 5555
STATE_DIM = 5
N_ACTIONS = 3

GAMMA = 0.95
LR = 1e-3
BATCH_SIZE = 64
REPLAY_CAPACITY = 5000
TARGET_UPDATE_EVERY = 400

EPSILON = 1.0
EPSILON_MIN = 0.05
EPSILON_DECAY = 0.995

# Message types
REQ_ACT = 1
TRANSITION = 2
RESET = 3

device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
print("Using device:", device)

# =========================
# Model
# =========================
class QNet(nn.Module):
    def __init__(self):
        super().__init__()
        self.net = nn.Sequential(
            nn.Linear(STATE_DIM, 32),
            nn.ReLU(),
            nn.Linear(32, N_ACTIONS)
        )
    def forward(self, x):
        return self.net(x)

online = QNet().to(device)
target = QNet().to(device)
target.load_state_dict(online.state_dict())

opt = optim.Adam(online.parameters(), lr=LR)
replay = deque(maxlen=REPLAY_CAPACITY)

step_count = 0

# =========================
# TCP helpers
# =========================
def recv_exact(conn, n):
    buf = b""
    while len(buf) < n:
        chunk = conn.recv(n - len(buf))
        if not chunk:
            raise ConnectionError("Client disconnected")
        buf += chunk
    return buf

def recv_msg(conn):
    # header: type (1 byte) + length (4 bytes)
    header = recv_exact(conn, 5)
    msg_type = header[0]
    payload_len = struct.unpack("!I", header[1:])[0]
    payload = recv_exact(conn, payload_len) if payload_len > 0 else b""
    return msg_type, payload

def send_action(conn, action_int):
    # reply: uint32 action
    conn.sendall(struct.pack("!I", int(action_int)))

# =========================
# RL helpers
# =========================
def choose_action(state_np):
    global EPSILON
    if random.random() < EPSILON:
        return random.randrange(N_ACTIONS)
    with torch.no_grad():
        s = torch.tensor(state_np, device=device).float().unsqueeze(0)  # (1,5)
        q = online(s)
        return int(torch.argmax(q, dim=1).item())

def train_step():
    if len(replay) < BATCH_SIZE:
        return

    batch = random.sample(replay, BATCH_SIZE)
    S, A, R, S2, D = map(np.array, zip(*batch))

    S  = torch.tensor(S,  device=device).float()            # (B,5)
    A  = torch.tensor(A,  device=device).long().unsqueeze(1) # (B,1)
    R  = torch.tensor(R,  device=device).float()            # (B,)
    S2 = torch.tensor(S2, device=device).float()            # (B,5)
    D  = torch.tensor(D,  device=device).float()            # (B,)

    q = online(S).gather(1, A).squeeze(1)                   # (B,)

    with torch.no_grad():
        q2 = target(S2).max(dim=1).values                   # (B,)
        y = R + GAMMA * q2 * (1.0 - D)

    loss = (q - y).pow(2).mean()

    opt.zero_grad()
    loss.backward()
    opt.step()

def maybe_update_target():
    global step_count
    if step_count % TARGET_UPDATE_EVERY == 0:
        target.load_state_dict(online.state_dict())

def decay_epsilon():
    global EPSILON
    if EPSILON > EPSILON_MIN:
        EPSILON *= EPSILON_DECAY

# =========================
# Server loop
# =========================
srv = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
srv.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
srv.bind((HOST, PORT))
srv.listen(1)
print(f"Listening on {HOST}:{PORT} ...")

conn, addr = srv.accept()
print("Connected:", addr)

try:
    while True:
        msg_type, payload = recv_msg(conn)

        if msg_type == REQ_ACT:
            # payload: STATE_DIM float32
            state = np.frombuffer(payload, dtype=np.float32)
            if state.shape[0] != STATE_DIM:
                raise ValueError(f"Bad state size: {state.shape}")
            action = choose_action(state)
            send_action(conn, action)

        elif msg_type == TRANSITION:
            # payload: s(5f) + a(u32) + r(f) + s2(5f) + done(u8)
            # Total bytes = 5*4 + 4 + 4 + 5*4 + 1 = 20 + 4 + 4 + 20 + 1 = 49
            if len(payload) != 49:
                raise ValueError(f"Bad transition payload len: {len(payload)}")

            s = np.frombuffer(payload[0:20], dtype=np.float32)
            a = struct.unpack("!I", payload[20:24])[0]
            r = struct.unpack("!f", payload[24:28])[0]
            s2 = np.frombuffer(payload[28:48], dtype=np.float32)
            done = payload[48]
            d = 1.0 if done != 0 else 0.0

            replay.append((s, a, r, s2, d))

            # train
            step_count += 1
            train_step()
            maybe_update_target()
            decay_epsilon()

        elif msg_type == RESET:
            # opcional: no es obligatorio
            pass

        else:
            raise ValueError(f"Unknown msg_type: {msg_type}")

except Exception as e:
    print("Stopped:", e)
finally:
    conn.close()
    srv.close()
