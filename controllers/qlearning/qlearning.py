from controller import Supervisor
import math, random, json, os

# =============== Hiperparámetros RL =================
TIME_STEP     = 64
ALPHA         = 0.18
GAMMA         = 0.96
EPSILON       = 0.25
EPSILON_MIN   = 0.06
EPSILON_DECAY = 0.9993

# =============== Cinemática e-puck ==================
FWD_SPEED  = 3.8
TURN_SPEED = 2.6
CURVE_DELTA = 0.9     # curvatura automática al ir junto a pared
NOISE_FWD   = 0.10    # ruido pequeño para romper ciclos

# =============== Sensores (umbrales) ================
PROX_THR_FRONT = 120.0
PROX_THR_SIDE  = 110.0
CORNER_THR     = 230.0

# =============== Meta/contacto ======================
ROBOT_RADIUS          = 0.035   # aprox radio e-puck
GOAL_CONFIRM_STEPS    = 10      # pasos para confirmar contacto
GOAL_CONFIRM_MIN_OK   = 8       # pasos que deben ser contacto
GOAL_COOLDOWN_STEPS   = 20      # espera antes de volver a contar (~1.2 s)
GOAL_LEAVE_MARGIN     = 0.015   # extra para exigir que se aleje antes de recontar

# =============== Anti-atasco ========================
STALL_WINDOW_STEPS = 140
UNSTICK_TURNS      = 18
UNSTICK_SPEED      = 2.4

# =============== Persistencia =======================
TABLE_FILE = "qtable_goal_current.json"

# =============== Utilidades Q-Table =================
ACTIONS = ["FORWARD", "LEFT", "RIGHT"]
Q = {}

def load_table():
    global Q
    if os.path.exists(TABLE_FILE):
        try:
            with open(TABLE_FILE, "r") as f:
                data = json.load(f)
            Q = {eval(k): v for k, v in data.items()}
        except:
            Q = {}

def save_table():
    try:
        with open(TABLE_FILE, "w") as f:
            json.dump({str(k): v for k, v in Q.items()}, f, indent=2)
    except:
        pass

def ensure_state(s):
    if s not in Q:
        Q[s] = {a: 0.0 for a in ACTIONS}

def choose_action(s):
    ensure_state(s)
    if random.random() < EPSILON:
        return random.choice(ACTIONS)
    return max(Q[s], key=Q[s].get)

def update_Q(s, a, r, s_next):
    ensure_state(s_next)
    Q[s][a] += ALPHA * (r + GAMMA * max(Q[s_next].values()) - Q[s][a])

# =============== Webots / Supervisor =================
robot = Supervisor()
dt = int(robot.getBasicTimeStep()) if robot.getBasicTimeStep() else TIME_STEP

left  = robot.getDevice("left wheel motor")
right = robot.getDevice("right wheel motor")
left.setPosition(float("inf")); right.setPosition(float("inf"))
left.setVelocity(0.0); right.setVelocity(0.0)

ps = [robot.getDevice(f"ps{i}") for i in range(8)]
for s in ps: s.enable(dt)

self_node   = robot.getSelf()
target_node = robot.getFromDef("TARGET")
if target_node is None:
    raise RuntimeError("No encuentro DEF TARGET (asegúrate que el cubo rojo tiene DEF=TARGET).")

self_tr   = self_node.getField("translation")
target_tr = target_node.getField("translation")

# --- obtener AABB del boundingObject (Box) del TARGET ---
bo_field = target_node.getField("boundingObject")
bo_node  = bo_field.getSFNode()
# El USE BOX1 apunta a un nodo Box; leemos su 'size'
box_size = bo_node.getField("size").getSFVec3f()  # [sx, sy, sz]
BOX_HALF_X = box_size[0] / 2.0
BOX_HALF_Z = box_size[2] / 2.0

def get_robot_xz():
    x, _, z = self_tr.getSFVec3f()
    return x, z

def get_target_xz():
    x, _, z = target_tr.getSFVec3f()
    return x, z

def distance(x1, z1, x2, z2):
    return math.hypot(x2 - x1, z2 - z1)

# --- contacto geométrico: robot (círculo) contra AABB del cubo (XY→ usamos XZ) ---
def circle_aabb_contact(rx, rz, cx, cz, hx, hz, rr):
    # clamp del centro del círculo al AABB
    closest_x = min(max(rx, cx - hx), cx + hx)
    closest_z = min(max(rz, cz - hz), cz + hz)
    dx = rx - closest_x
    dz = rz - closest_z
    return (dx*dx + dz*dz) <= (rr*rr)

# --- estado por sensores ---
def sense():
    left_val  = ps[5].getValue() + ps[6].getValue()
    front_val = ps[0].getValue() + ps[7].getValue()
    right_val = ps[1].getValue() + ps[2].getValue()
    s_left   = 1 if left_val  > PROX_THR_SIDE  else 0
    s_front  = 1 if front_val > PROX_THR_FRONT else 0
    s_right  = 1 if right_val > PROX_THR_SIDE  else 0
    s_corner = 1 if (left_val > CORNER_THR and right_val > CORNER_THR) else 0
    return (s_left, s_front, s_right, s_corner), (left_val, front_val, right_val)

# --- ejecutar acción (avance curvado junto a pared) ---
def act(action, sensed_raw):
    left_val, front_val, right_val = sensed_raw
    noise = (random.random() - 0.5) * 2 * NOISE_FWD
    if action == "FORWARD":
        vl = FWD_SPEED + noise
        vr = FWD_SPEED - noise
        if right_val > PROX_THR_SIDE and left_val <= PROX_THR_SIDE:
            vl -= CURVE_DELTA; vr += CURVE_DELTA
        elif left_val > PROX_THR_SIDE and right_val <= PROX_THR_SIDE:
            vl += CURVE_DELTA; vr -= CURVE_DELTA
        left.setVelocity(vl); right.setVelocity(vr)
    elif action == "LEFT":
        left.setVelocity(-TURN_SPEED); right.setVelocity(TURN_SPEED)
    elif action == "RIGHT":
        left.setVelocity(TURN_SPEED);  right.setVelocity(-TURN_SPEED)

# --- recompensa ---
def compute_reward(next_state, dist_prev, dist_now):
    s_left, s_front, s_right, s_corner = next_state
    r = -0.02
    if s_front == 1:  r -= 4.0
    if s_left or s_right: r -= 0.4
    if s_corner == 1:  r -= 1.5
    if dist_prev is not None:
        delta = dist_prev - dist_now
        r += 2.0 * delta
        if delta < -1e-4:
            r -= 0.6
    # pequeño bonus por cercanía (no imprime nada; solo shaping)
    if dist_now < (max(BOX_HALF_X, BOX_HALF_Z) + ROBOT_RADIUS + 0.01):
        r += 1.0
    return r

# =============== Bucle RL ============================
load_table()
best_dist = float("inf")
stalled   = 0
unstick   = 0
goal_count = 0

# hysteresis / cooldown para llegada
goal_locked   = False
goal_cooldown = 0

while robot.step(dt) != -1:
    state, raw = sense()
    rx, rz = get_robot_xz()
    tx, tz = get_target_xz()
    dist_now = distance(rx, rz, tx, tz)

    # Anti-atasco por falta de mejora
    if dist_now + 1e-4 < best_dist:
        best_dist = dist_now
        stalled = 0
    else:
        stalled += 1

    if unstick > 0:
        sgn = -1 if random.random() < 0.5 else 1
        left.setVelocity(-UNSTICK_SPEED * sgn)
        right.setVelocity(UNSTICK_SPEED * sgn)
        unstick -= 1
        continue

    if stalled >= STALL_WINDOW_STEPS:
        unstick = UNSTICK_TURNS
        stalled = 0
        EPSILON = min(0.35, EPSILON + 0.10)
        continue

    # Elegir y ejecutar acción
    action = choose_action(state)
    act(action, raw)

    # Observar siguiente estado
    robot.step(dt)
    next_state, _ = sense()
    rx2, rz2 = get_robot_xz()
    dist_next = distance(rx2, rz2, tx, tz)

    # Recompensa + actualización
    r = compute_reward(next_state, dist_now, dist_next)
    update_Q(state, action, r, next_state)

    # ======= Llegada por CONTACTO GEOMÉTRICO =======
    # Contacto si el círculo (robot) colisiona con el AABB del cubo
    contact_now = circle_aabb_contact(rx2, rz2, tx, tz, BOX_HALF_X, BOX_HALF_Z, ROBOT_RADIUS)

    if goal_cooldown > 0:
        goal_cooldown -= 1
    else:
        if goal_locked:
            # desbloquear solo si se aleja claramente de la caja
            # usamos distancia centro-centro respecto al "radio externo" (semi-diagonal + margen)
            leave_r = math.hypot(BOX_HALF_X, BOX_HALF_Z) + ROBOT_RADIUS + GOAL_LEAVE_MARGIN
            if dist_next > leave_r:
                goal_locked = False
        else:
            if contact_now:
                # Confirmación temporal (debe mantener el contacto la mayoría de pasos)
                ok = 0
                for _ in range(GOAL_CONFIRM_STEPS):
                    robot.step(dt)
                    rxc, rzc = get_robot_xz()
                    contact_c = circle_aabb_contact(rxc, rzc, tx, tz, BOX_HALF_X, BOX_HALF_Z, ROBOT_RADIUS)
                    if contact_c:
                        ok += 1
                if ok >= GOAL_CONFIRM_MIN_OK:
                    # ------- META ALCANZADA (una sola vez) -------
                    goal_count += 1
                    print(f"\nMeta alcanzada! Total: {goal_count} veces\n")

                    # Pausa visible 1 s
                    left.setVelocity(0.0); right.setVelocity(0.0)
                    for _ in range(int(1000 // dt)):
                        robot.step(dt)

                    # salida aleatoria breve para separarse del cubo
                    for _ in range(10):
                        sgn = -1 if random.random() < 0.5 else 1
                        left.setVelocity(2.0 * sgn)
                        right.setVelocity(-2.0 * sgn)
                        robot.step(dt)

                    # activar hysteresis y cooldown
                    goal_locked   = True
                    goal_cooldown = GOAL_COOLDOWN_STEPS

                    # reset métricas de atasco
                    best_dist = float("inf")
                    stalled   = 0
                    continue

    # Decaimiento ε y guardado periódico
    EPSILON = max(EPSILON_MIN, EPSILON * EPSILON_DECAY)
    if random.random() < 0.02:
        save_table()
