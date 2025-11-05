import argparse, socket, time, math
from collections import deque
import cv2, numpy as np
import mediapipe as mp

# ---------------- Args ----------------
p = argparse.ArgumentParser()
p.add_argument("--ip",  type=str, required=True, help="ESP32 IPv4 from Serial Monitor")
p.add_argument("--port",type=int, default=4210)
p.add_argument("--cam", type=int, default=None, help="Camera index override (0/1/2/3)")
args = p.parse_args()

TARGET_IP, TARGET_PORT = args.ip, args.port

# ---------------- UDP ----------------
sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
sock.setblocking(False)
SEND_MIN_INTERVAL = 0.02   # 50 Hz
MIN_DELTA_DEG = 1
last_send_ts = 0.0
last_sent_s1 = None
last_sent_s2 = None

# ---------------- Mapping/UI ----------------
SMOOTH_N = 7
PINCH_FORCE_ZERO = 0.08
ZERO_EPS = 0.02
R_MIN_DEFAULT, R_MAX_DEFAULT = 0.10, 0.60
ANGLE_MIN, ANGLE_MAX = 0, 180
BAR_H, BAR_W, MARGIN = 220, 22, 40

# ---------------- MediaPipe ----------------
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils
mp_styles  = mp.solutions.drawing_styles
hands = mp_hands.Hands(False, 2, 1, 0.6, 0.6)

state = {
    "Left":  {"r_min": R_MIN_DEFAULT, "r_max": R_MAX_DEFAULT, "hist": deque(maxlen=SMOOTH_N), "angle": 0.0, "r": None},
    "Right": {"r_min": R_MIN_DEFAULT, "r_max": R_MAX_DEFAULT, "hist": deque(maxlen=SMOOTH_N), "angle": 0.0, "r": None},
}

def l2(p1,p2): return math.hypot(p1[0]-p2[0], p1[1]-p2[1])
def clamp(x, lo, hi): return max(lo, min(hi, x))
def map_norm_to_angle(r, rmin, rmax):
    if r is None or np.isnan(r): return None
    if r <= PINCH_FORCE_ZERO:    return 0.0
    t = (r - rmin) / max(1e-6, (rmax - rmin))
    t = clamp(t, 0.0, 1.0)
    return ANGLE_MIN + t * (ANGLE_MAX - ANGLE_MIN)

def put_text(img, text, org, scale=0.8, thickness=2, color=(255,255,255)):
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (0,0,0), thickness+2, cv2.LINE_AA)
    cv2.putText(img, text, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color, thickness, cv2.LINE_AA)

def draw_bar(img, angle, x, y, label):
    cv2.rectangle(img, (x, y), (x+BAR_W, y+BAR_H), (255,255,255), 2)
    ang = 0.0 if angle is None else float(angle)
    fill_h = int((ang/180.0)*(BAR_H-4))
    cv2.rectangle(img, (x+2, y+BAR_H-2-fill_h), (x+BAR_W-2, y+BAR_H-2), (255,255,255), -1)
    for k in [0,45,90,135,180]:
        ty = y + BAR_H - int((k/180.0)*(BAR_H-4)) - 2
        cv2.line(img, (x+BAR_W+6, ty), (x+BAR_W+26, ty), (255,255,255), 1)
        put_text(img, f"{k}", (x+BAR_W+30, ty+4), 0.5, 1)
    put_text(img, label, (x-2, y-10), 0.7, 2)

def process_hand(frame, label, lm_list, w, h):
    idxs = [4,8,5,17]
    pts = {i:(int(lm_list[i].x*w), int(lm_list[i].y*h)) for i in idxs}
    d_tip = l2(pts[4], pts[8])
    d_ref = max(10.0, l2(pts[5], pts[17]))
    r = d_tip/d_ref
    s = state[label]
    angle = map_norm_to_angle(r, s["r_min"], s["r_max"])
    if angle is None: return
    s["hist"].append(angle)
    s["angle"] = float(np.mean(s["hist"])) if s["hist"] else angle
    s["r"] = r
    # visuals
    cv2.circle(frame, pts[4], 10, (0,255,255), -1)
    cv2.circle(frame, pts[8], 10, (0,255,255), -1)
    cv2.line(frame, pts[4], pts[8], (255,255,255), 2)
    cv2.circle(frame, pts[5], 6, (255,255,255), -1)
    cv2.circle(frame, pts[17],6, (255,255,255), -1)
    cv2.line(frame, pts[5], pts[17], (200,200,200), 1)

def label_hands_safely(res):
    labeled=[]
    if not res or not res.multi_hand_landmarks: return labeled
    if res.multi_handedness and len(res.multi_handedness)==len(res.multi_hand_landmarks):
        for hinfo,lm in zip(res.multi_handedness,res.multi_hand_landmarks):
            labeled.append((hinfo.classification[0].label,lm))
        return labeled
    # fallback by wrist-x order
    hands_by_x=[(lm.landmark[0].x,lm) for lm in res.multi_hand_landmarks]
    hands_by_x.sort(key=lambda t:t[0])
    if len(hands_by_x)==1: labeled.append(("Right",hands_by_x[0][1]))
    else:
        labeled.append(("Right",hands_by_x[0][1])); labeled.append(("Left",hands_by_x[1][1]))
    return labeled

# ---------- Camera init (robust) ----------
def open_camera(idx_hint=None):
    indices = [idx_hint] if idx_hint is not None else [0,1,2,3]
    for i in indices:
        cap = cv2.VideoCapture(i, cv2.CAP_DSHOW)  # CAP_DSHOW helps on Windows
        cap.set(cv2.CAP_PROP_FRAME_WIDTH,  1280)
        cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 720)
        if cap.isOpened():
            ret, _ = cap.read()
            if ret:
                print(f"[INFO] Camera opened on index {i}")
                return cap
        if cap: cap.release()
    raise RuntimeError("No camera opened. Try --cam 0/1/2/3 and close apps using the camera.")

%cap = open_camera(args.cam)
%cv2.namedWindow("Dual Pinch-to-Servo (Two Hands + UDP/STA)", cv2.WINDOW_NORMAL)
cap = open_camera(args.cam)
cv2.namedWindow("Dual Pinch-to-Servo (Two Hands + UDP/STA)", cv2.WINDOW_NORMAL)
cv2.resizeWindow("Dual Pinch-to-Servo (Two Hands + UDP/STA)", 1280, 720)
cv2.moveWindow("Dual Pinch-to-Servo (Two Hands + UDP/STA)", 100, 50)


both_zero = False

try:
    while True:
        ok, frame = cap.read()
        if not ok:
            print("[WARN] Camera read failed. Reopening…")
            cap.release(); time.sleep(0.4); cap = open_camera(args.cam); continue

        frame = cv2.flip(frame,1)
        H,W = frame.shape[:2]
        rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        res = hands.process(rgb)

        state["Left"]["r"]=None; state["Right"]["r"]=None

        for label,lm in label_hands_safely(res):
            if label not in ("Left","Right"): label="Right"
            mp_drawing.draw_landmarks(frame,lm, mp_hands.HAND_CONNECTIONS,
                                      mp_styles.get_default_hand_landmarks_style(),
                                      mp_styles.get_default_hand_connections_style())
            process_hand(frame,label,lm.landmark,W,H)

        left_ok  = state["Left"]["r"]  is not None
        right_ok = state["Right"]["r"] is not None
        left_angle  = int(round(state["Left"]["angle"]))  if left_ok  else None
        right_angle = int(round(state["Right"]["angle"])) if right_ok else None

        draw_bar(frame, state["Left"]["angle"],  MARGIN,                 MARGIN, "Left → Servo 2")
        draw_bar(frame, state["Right"]["angle"], W-MARGIN-BAR_W-40,      MARGIN, "Right → Servo 1")

        if left_ok:  put_text(frame,  f"Left:  {left_angle:3d}°  r={state['Left']['r']:.3f}",  (MARGIN, H-60))
        else:        put_text(frame,  "Left:  --- (show LEFT hand)",                            (MARGIN, H-60))
        if right_ok: put_text(frame, f"Right: {right_angle:3d}°  r={state['Right']['r']:.3f}", (MARGIN, H-30))
        else:        put_text(frame, "Right: --- (show RIGHT hand)",                            (MARGIN, H-30))
        put_text(frame, "Calib Left: Z=ZERO X=MAX | Right: N=ZERO M=MAX | Q=Quit", (MARGIN, 30), 0.7, 1)

        # ------- STRICT TWO-HAND SEND + both-zero latch -------
        now = time.time()
        if left_ok and right_ok and (now - last_send_ts >= SEND_MIN_INTERVAL):
            l_touch = state["Left"]["r"]  <= (PINCH_FORCE_ZERO + ZERO_EPS)
            r_touch = state["Right"]["r"] <= (PINCH_FORCE_ZERO + ZERO_EPS)

            if l_touch and r_touch:       # <-- FIXED 'and' (Python)
                s1, s2 = 0, 0
                send = (last_sent_s1 != 0 or last_sent_s2 != 0)
                both_zero = True
            else:
                s1 = right_angle
                s2 = left_angle
                send = (last_sent_s1 is None or abs(s1-last_sent_s1)>=MIN_DELTA_DEG or
                        last_sent_s2 is None or abs(s2-last_sent_s2)>=MIN_DELTA_DEG)
                both_zero = False

            if send:
                pkt = f"{s1},{s2}\n".encode("utf-8")
                try:
                    sock.sendto(pkt, (TARGET_IP, TARGET_PORT))
                    print("[TX-UDP]", s1, s2, "(both-zero)" if (s1==0 and s2==0) else "")
                    last_sent_s1, last_sent_s2 = s1, s2
                    last_send_ts = now
                except Exception as e:
                    print("[UDP WARN]", e)

        cv2.imshow("Dual Pinch-to-Servo (Two Hands + UDP/STA)", frame)
        k = cv2.waitKey(1) & 0xFF
        if k == ord('q'): break
        if k == ord('z') and state["Left"]["r"] is not None:
            r_cur = state["Left"]["r"];  state["Left"]["r_min"] = max(0.0, min(r_cur, state["Left"]["r_max"] - 0.02))
        elif k == ord('x') and state["Left"]["r"] is not None:
            r_cur = state["Left"]["r"];  state["Left"]["r_max"] = max(state["Left"]["r_min"] + 0.02, r_cur)
        elif k == ord('n') and state["Right"]["r"] is not None:
            r_cur = state["Right"]["r"]; state["Right"]["r_min"] = max(0.0, min(r_cur, state["Right"]["r_max"] - 0.02))
        elif k == ord('m') and state["Right"]["r"] is not None:
            r_cur = state["Right"]["r"]; state["Right"]["r_max"] = max(state["Right"]["r_min"] + 0.02, r_cur)

finally:
    cap.release(); hands.close(); sock.close(); cv2.destroyAllWindows()
