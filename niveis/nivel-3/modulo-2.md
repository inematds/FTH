---
layout: page
title: "Módulo 3.2: Visão Computacional Avançada"
permalink: /niveis/nivel-3/modulo-2/
---

# 👁️ Módulo 3.2: Visão Computacional Avançada

**Dê olhos ao seu robô: percepção visual para robótica!**

---

## 📋 Informações do Módulo

| Informação | Detalhes |
|------------|----------|
| **Duração estimada** | 15-20 horas |
| **Nível** | Avançado |
| **Pré-requisitos** | Python, RL básico, álgebra linear |
| **Ferramentas** | OpenCV, PyTorch, YOLO, MediaPipe, ROS 2 |

---

## 🎯 Objetivos de Aprendizado

Ao completar este módulo, você será capaz de:

- [ ] Processar imagens em tempo real para robótica
- [ ] Implementar detecção de objetos com YOLO
- [ ] Realizar segmentação semântica e de instâncias
- [ ] Estimar poses humanas e de objetos
- [ ] Integrar visão computacional com controle de robôs
- [ ] Treinar modelos de visão personalizados
- [ ] Lidar com o Sim2Real gap em visão

---

## 📚 Conteúdo Teórico

### 1. Fundamentos de Visão Computacional para Robótica

#### Por que Visão é Crítica para Robôs?

Robôs humanoides precisam **perceber o mundo** para:
- 🎯 **Manipulação**: Pegar objetos desconhecidos
- 🚶 **Navegação**: Evitar obstáculos dinâmicos
- 🤝 **Interação**: Detectar pessoas e gestos
- 🎭 **Imitação**: Aprender observando humanos

**Diferença: Visão para Robótica vs. Computer Vision tradicional**

| Aspecto | CV Tradicional | CV para Robótica |
|---------|---------------|------------------|
| **Latência** | Segundos OK | < 50ms crítico |
| **Precisão** | 99%+ necessário | 90% + robustez OK |
| **Ambiente** | Controlado | Iluminação variável |
| **Output** | Labels/bboxes | Coordenadas 3D |
| **Integração** | Standalone | ROS 2 + controle |

#### Pipeline Clássico de Visão Robótica

```
Câmera → Pré-processamento → Detecção/Segmentação →
Transformação 3D → Planejamento → Controle
```

**Exemplo: Pegar uma caneca**

```python
import cv2
import numpy as np

def robot_pick_pipeline(image, camera_intrinsics, robot):
    """Pipeline completo de visão para manipulação"""

    # 1. PRÉ-PROCESSAMENTO
    image_rgb = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)
    image_normalized = image_rgb / 255.0

    # 2. DETECÇÃO DE OBJETOS
    detections = detect_objects(image_normalized)  # YOLO
    mug_bbox = [d for d in detections if d['class'] == 'mug'][0]

    # 3. SEGMENTAÇÃO (opcional, para grasp preciso)
    mask = segment_object(image, mug_bbox)

    # 4. ESTIMAÇÃO DE POSE 3D
    depth_map = get_depth(image)  # De câmera RGB-D ou estéreo
    point_cloud = depth_to_pointcloud(depth_map, camera_intrinsics)
    mug_pose_3d = estimate_3d_pose(point_cloud, mask)

    # 5. TRANSFORMAÇÃO PARA FRAME DO ROBÔ
    mug_pose_robot_frame = transform_camera_to_robot(
        mug_pose_3d,
        robot.camera_transform
    )

    # 6. PLANEJAMENTO DE GRASP
    grasp_pose = plan_grasp(mug_pose_robot_frame, mug_shape='cylinder')

    # 7. EXECUÇÃO
    robot.move_to(grasp_pose)
    robot.close_gripper()

    return True
```

---

### 2. Processamento de Imagens Avançado

#### Calibração de Câmera

Antes de qualquer coisa, **calibre sua câmera** para ter métricas precisas.

```python
import cv2
import numpy as np
import glob

def calibrate_camera(images_path):
    """
    Calibra câmera usando padrão de xadrez
    """
    # Preparar pontos do mundo real (0,0,0), (1,0,0), ...
    pattern_size = (9, 6)  # Cantos internos do xadrez
    square_size = 0.025  # 25mm por quadrado

    objp = np.zeros((pattern_size[0] * pattern_size[1], 3), np.float32)
    objp[:, :2] = np.mgrid[0:pattern_size[0], 0:pattern_size[1]].T.reshape(-1, 2)
    objp *= square_size

    # Arrays para armazenar pontos
    objpoints = []  # Pontos 3D no mundo real
    imgpoints = []  # Pontos 2D na imagem

    # Processar imagens de calibração
    images = glob.glob(f'{images_path}/*.jpg')

    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # Encontrar cantos do xadrez
        ret, corners = cv2.findChessboardCorners(gray, pattern_size, None)

        if ret:
            objpoints.append(objp)
            # Refinar localização dos cantos
            corners_refined = cv2.cornerSubPix(
                gray, corners, (11, 11), (-1, -1),
                criteria=(cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            )
            imgpoints.append(corners_refined)

    # Calibrar câmera
    ret, camera_matrix, dist_coeffs, rvecs, tvecs = cv2.calibrateCamera(
        objpoints, imgpoints, gray.shape[::-1], None, None
    )

    print(f"Matriz da câmera:\n{camera_matrix}")
    print(f"Coeficientes de distorção:\n{dist_coeffs}")

    return camera_matrix, dist_coeffs

# Usar calibração para undistort imagens
camera_matrix, dist_coeffs = calibrate_camera('./calibration_images/')

def undistort_image(image, camera_matrix, dist_coeffs):
    h, w = image.shape[:2]
    new_camera_matrix, roi = cv2.getOptimalNewCameraMatrix(
        camera_matrix, dist_coeffs, (w, h), 1, (w, h)
    )
    undistorted = cv2.undistort(image, camera_matrix, dist_coeffs, None, new_camera_matrix)
    return undistorted
```

#### Segmentação e Tracking

**Segmentação de cor (simples, mas efetivo):**

```python
def segment_by_color(image, lower_hsv, upper_hsv):
    """
    Segmenta objetos por cor (ex: bola vermelha)
    """
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    mask = cv2.inRange(hsv, lower_hsv, upper_hsv)

    # Operações morfológicas para limpar ruído
    kernel = np.ones((5, 5), np.uint8)
    mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
    mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

    # Encontrar contornos
    contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

    if len(contours) > 0:
        # Maior contorno = objeto de interesse
        largest_contour = max(contours, key=cv2.contourArea)
        x, y, w, h = cv2.boundingRect(largest_contour)
        center = (x + w//2, y + h//2)
        return mask, center

    return mask, None

# Exemplo: Detectar bola vermelha
lower_red = np.array([0, 100, 100])
upper_red = np.array([10, 255, 255])
mask, center = segment_by_color(image, lower_red, upper_red)
```

**Tracking de objetos (KCF Tracker):**

```python
import cv2

# Inicializar tracker
tracker = cv2.TrackerKCF_create()

# Primeiro frame: selecionar objeto
ret, frame = cap.read()
bbox = cv2.selectROI("Frame", frame, False)
tracker.init(frame, bbox)

# Loop de tracking
while True:
    ret, frame = cap.read()
    if not ret:
        break

    # Atualizar tracker
    success, bbox = tracker.update(frame)

    if success:
        x, y, w, h = [int(v) for v in bbox]
        cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)
    else:
        cv2.putText(frame, "Tracking falhou", (10, 50),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

    cv2.imshow("Tracking", frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break
```

#### Transformações Geométricas 3D

Converter pontos da imagem 2D para coordenadas 3D no mundo real:

```python
def pixel_to_3d_point(u, v, depth, camera_matrix):
    """
    Converte pixel (u, v) com profundidade para ponto 3D

    Parâmetros:
    - u, v: coordenadas do pixel
    - depth: profundidade em metros
    - camera_matrix: matriz intrínseca da câmera
    """
    fx = camera_matrix[0, 0]
    fy = camera_matrix[1, 1]
    cx = camera_matrix[0, 2]
    cy = camera_matrix[1, 2]

    # Fórmula de projeção inversa
    x = (u - cx) * depth / fx
    y = (v - cy) * depth / fy
    z = depth

    return np.array([x, y, z])

# Exemplo de uso
camera_matrix = np.array([
    [500, 0, 320],
    [0, 500, 240],
    [0, 0, 1]
])

# Objeto detectado no pixel (320, 240) com profundidade 1.5m
point_3d = pixel_to_3d_point(320, 240, 1.5, camera_matrix)
print(f"Posição 3D: {point_3d}")  # [0, 0, 1.5] (centro da imagem)
```

---

### 3. Detecção de Objetos com YOLO

#### O que é YOLO?

**YOLO** (You Only Look Once) é a arquitetura state-of-the-art para detecção de objetos em tempo real.

**Vantagens para robótica:**
- ⚡ **Rápido**: 30-60 FPS em GPU
- 🎯 **Preciso**: mAP > 50% no COCO
- 🔧 **Flexível**: Fácil de treinar em datasets customizados

**Paper original:** [YOLOv1 (Redmon et al., 2016)](https://arxiv.org/abs/1506.02640)
**Versão moderna:** [YOLOv8 (Ultralytics, 2023)](https://github.com/ultralytics/ultralytics)

#### Instalação e Uso Rápido

```bash
pip install ultralytics
```

```python
from ultralytics import YOLO
import cv2

# 1. Carregar modelo pré-treinado
model = YOLO('yolov8n.pt')  # n=nano (rápido), s=small, m=medium, l=large

# 2. Detecção em imagem
image = cv2.imread('scene.jpg')
results = model(image)

# 3. Processar resultados
for result in results:
    boxes = result.boxes  # Bounding boxes
    for box in boxes:
        # Coordenadas
        x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
        # Confiança
        conf = box.conf[0].cpu().item()
        # Classe
        cls = int(box.cls[0].cpu().item())
        class_name = model.names[cls]

        # Desenhar
        cv2.rectangle(image, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
        cv2.putText(image, f"{class_name} {conf:.2f}",
                    (int(x1), int(y1)-10),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

cv2.imshow('Detections', image)
cv2.waitKey(0)
```

#### Detecção em Tempo Real (Webcam/ROS)

```python
import cv2
from ultralytics import YOLO

def real_time_detection():
    model = YOLO('yolov8n.pt')
    cap = cv2.VideoCapture(0)  # Webcam

    while True:
        ret, frame = cap.read()
        if not ret:
            break

        # Inferência
        results = model(frame, verbose=False)

        # Visualizar
        annotated_frame = results[0].plot()

        cv2.imshow('YOLOv8 Real-Time', annotated_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

real_time_detection()
```

#### Treinar YOLO em Dataset Customizado

**Caso de uso: Detectar peças específicas de robô**

**1. Preparar dataset (formato YOLO):**

```
dataset/
├── images/
│   ├── train/
│   │   ├── img1.jpg
│   │   └── img2.jpg
│   └── val/
│       ├── img3.jpg
│       └── img4.jpg
└── labels/
    ├── train/
    │   ├── img1.txt
    │   └── img2.txt
    └── val/
        ├── img3.txt
        └── img4.txt
```

**Formato de label (img1.txt):**
```
0 0.5 0.5 0.3 0.4
1 0.7 0.3 0.2 0.2
```
(class_id, x_center, y_center, width, height) - normalizado [0-1]

**2. Criar arquivo de configuração (data.yaml):**

```yaml
path: ./dataset
train: images/train
val: images/val

names:
  0: gear
  1: screw
  2: motor
```

**3. Treinar:**

```python
from ultralytics import YOLO

# Carregar modelo base
model = YOLO('yolov8n.pt')

# Fine-tuning
results = model.train(
    data='data.yaml',
    epochs=100,
    imgsz=640,
    batch=16,
    lr0=0.01,
    device=0,  # GPU
    project='robot_parts_detector',
    name='exp1'
)

# Validar
metrics = model.val()
print(f"mAP50: {metrics.box.map50}")
print(f"mAP50-95: {metrics.box.map}")

# Exportar para produção
model.export(format='onnx')  # Ou 'torchscript', 'tflite'
```

**4. Usar modelo treinado:**

```python
model = YOLO('robot_parts_detector/exp1/weights/best.pt')
results = model('test_image.jpg')
```

#### YOLO + Depth para Localização 3D

```python
import numpy as np
import cv2
from ultralytics import YOLO

class ObjectLocalizer3D:
    def __init__(self, yolo_model_path, camera_matrix):
        self.model = YOLO(yolo_model_path)
        self.camera_matrix = camera_matrix

    def detect_and_localize(self, rgb_image, depth_image):
        """
        Detecta objetos e retorna posições 3D
        """
        results = self.model(rgb_image)
        objects_3d = []

        for box in results[0].boxes:
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            cls = int(box.cls[0])
            conf = box.conf[0].cpu().item()

            # Centro da bbox
            u = int((x1 + x2) / 2)
            v = int((y1 + y2) / 2)

            # Profundidade média na região
            depth_roi = depth_image[int(y1):int(y2), int(x1):int(x2)]
            depth = np.median(depth_roi)

            # Converter para 3D
            point_3d = pixel_to_3d_point(u, v, depth, self.camera_matrix)

            objects_3d.append({
                'class': self.model.names[cls],
                'confidence': conf,
                'bbox_2d': (x1, y1, x2, y2),
                'position_3d': point_3d
            })

        return objects_3d

# Uso
localizer = ObjectLocalizer3D('yolov8n.pt', camera_matrix)
objects = localizer.detect_and_localize(rgb_frame, depth_frame)

for obj in objects:
    print(f"{obj['class']} at {obj['position_3d']} with conf {obj['confidence']:.2f}")
```

---

### 4. Segmentação Semântica e de Instâncias

#### Diferença entre Segmentação

- **Semântica**: Classifica cada pixel (ex: "pessoa", "cadeira")
- **Instâncias**: Separa objetos individuais (ex: "pessoa 1", "pessoa 2")

#### Segmentação com YOLOv8-seg

```python
from ultralytics import YOLO

# Modelo com segmentação
model = YOLO('yolov8n-seg.pt')

# Inferência
results = model('image.jpg')

# Acessar máscaras
for result in results:
    masks = result.masks  # Máscaras de segmentação
    boxes = result.boxes

    for i, (mask, box) in enumerate(zip(masks.data, boxes)):
        # Máscara binária
        mask_np = mask.cpu().numpy()

        # Classe
        cls = int(box.cls[0])
        class_name = model.names[cls]

        # Salvar máscara
        cv2.imwrite(f'mask_{i}_{class_name}.png', mask_np * 255)
```

#### Segmentação Avançada com Detectron2

**Detectron2** (Facebook AI) oferece modelos state-of-the-art:

```bash
pip install 'git+https://github.com/facebookresearch/detectron2.git'
```

```python
from detectron2 import model_zoo
from detectron2.engine import DefaultPredictor
from detectron2.config import get_cfg
from detectron2.utils.visualizer import Visualizer
from detectron2.data import MetadataCatalog
import cv2

# Configurar
cfg = get_cfg()
cfg.merge_from_file(model_zoo.get_config_file("COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml"))
cfg.MODEL.ROI_HEADS.SCORE_THRESH_TEST = 0.5
cfg.MODEL.WEIGHTS = model_zoo.get_checkpoint_url("COCO-InstanceSegmentation/mask_rcnn_R_50_FPN_3x.yaml")

predictor = DefaultPredictor(cfg)

# Inferência
im = cv2.imread("input.jpg")
outputs = predictor(im)

# Visualizar
v = Visualizer(im[:, :, ::-1], MetadataCatalog.get(cfg.DATASETS.TRAIN[0]), scale=1.2)
out = v.draw_instance_predictions(outputs["instances"].to("cpu"))
cv2.imshow('Segmentation', out.get_image()[:, :, ::-1])
cv2.waitKey(0)

# Acessar dados
instances = outputs["instances"]
masks = instances.pred_masks  # torch.Tensor (N, H, W)
boxes = instances.pred_boxes
classes = instances.pred_classes
scores = instances.scores
```

#### Aplicação: Grasp Planning com Segmentação

```python
def plan_grasp_from_segmentation(rgb_image, depth_image, target_class='cup'):
    """
    Usa segmentação para planejar grasp em objeto específico
    """
    # 1. Segmentar objetos
    outputs = predictor(rgb_image)
    instances = outputs["instances"]

    # 2. Filtrar classe desejada
    target_idx = [i for i, cls in enumerate(instances.pred_classes)
                  if model_metadata.thing_classes[cls] == target_class]

    if not target_idx:
        return None

    # 3. Pegar primeira instância da classe
    idx = target_idx[0]
    mask = instances.pred_masks[idx].cpu().numpy()

    # 4. Calcular centroide da máscara
    y_coords, x_coords = np.where(mask)
    centroid_x = int(np.mean(x_coords))
    centroid_y = int(np.mean(y_coords))

    # 5. Obter profundidade
    depth = depth_image[centroid_y, centroid_x]

    # 6. Converter para 3D
    grasp_point_3d = pixel_to_3d_point(centroid_x, centroid_y, depth, camera_matrix)

    # 7. Estimar orientação (usando PCA nos pontos da máscara)
    points_3d = []
    for y, x in zip(y_coords[::10], x_coords[::10]):  # Subsample
        d = depth_image[y, x]
        p = pixel_to_3d_point(x, y, d, camera_matrix)
        points_3d.append(p)

    points_3d = np.array(points_3d)
    # PCA para encontrar eixo principal
    from sklearn.decomposition import PCA
    pca = PCA(n_components=3)
    pca.fit(points_3d)
    principal_axis = pca.components_[0]  # Eixo principal

    return {
        'position': grasp_point_3d,
        'orientation': principal_axis,
        'mask': mask
    }
```

---

### 5. Pose Estimation (Humanos e Objetos)

#### Estimação de Pose Humana com MediaPipe

**MediaPipe** (Google) oferece pose estimation em tempo real:

```bash
pip install mediapipe
```

```python
import mediapipe as mp
import cv2

mp_pose = mp.solutions.pose
mp_drawing = mp.solutions.drawing_utils

# Inicializar
pose = mp_pose.Pose(
    static_image_mode=False,
    model_complexity=1,
    smooth_landmarks=True,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5
)

cap = cv2.VideoCapture(0)

while cap.isOpened():
    ret, frame = cap.read()
    if not ret:
        break

    # Converter para RGB
    image_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

    # Processar
    results = pose.process(image_rgb)

    # Desenhar landmarks
    if results.pose_landmarks:
        mp_drawing.draw_landmarks(
            frame,
            results.pose_landmarks,
            mp_pose.POSE_CONNECTIONS
        )

        # Acessar coordenadas
        landmarks = results.pose_landmarks.landmark
        left_wrist = landmarks[mp_pose.PoseLandmark.LEFT_WRIST]
        print(f"Punho esquerdo: x={left_wrist.x}, y={left_wrist.y}, z={left_wrist.z}")

    cv2.imshow('Pose Estimation', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
```

#### Aplicação: Robô Imitando Gestos

```python
class GestureImitator:
    def __init__(self, robot_controller):
        self.pose = mp_pose.Pose()
        self.robot = robot_controller

    def track_and_imitate(self, frame):
        """
        Detecta pose humana e replica no robô
        """
        image_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.pose.process(image_rgb)

        if not results.pose_landmarks:
            return

        landmarks = results.pose_landmarks.landmark

        # Mapear ombro e cotovelo
        left_shoulder = landmarks[mp_pose.PoseLandmark.LEFT_SHOULDER]
        left_elbow = landmarks[mp_pose.PoseLandmark.LEFT_ELBOW]
        left_wrist = landmarks[mp_pose.PoseLandmark.LEFT_WRIST]

        # Calcular ângulo do cotovelo
        angle = self.calculate_angle(left_shoulder, left_elbow, left_wrist)

        # Enviar para robô
        self.robot.set_joint_angle('left_elbow', angle)

    def calculate_angle(self, a, b, c):
        """Calcula ângulo ABC"""
        import math
        radians = math.atan2(c.y - b.y, c.x - b.x) - math.atan2(a.y - b.y, a.x - b.x)
        angle = np.abs(radians * 180.0 / np.pi)
        if angle > 180.0:
            angle = 360 - angle
        return angle
```

#### Pose Estimation de Objetos (6D Pose)

Para manipulação robótica precisa, precisamos estimar **6DOF pose** (3D posição + 3D orientação):

```python
# Usando PVNet ou similar (exemplo conceitual)
def estimate_6d_pose(rgb_image, object_model):
    """
    Estima pose 6D de objeto conhecido
    """
    # 1. Detectar objeto (YOLO)
    bbox = detect_object(rgb_image, object_class)

    # 2. Extrair keypoints (rede neural treinada)
    keypoints_2d = extract_keypoints(rgb_image, bbox)

    # 3. Resolver PnP (Perspective-n-Point)
    # Mapear keypoints 2D para 3D usando modelo CAD
    success, rvec, tvec = cv2.solvePnP(
        object_model.keypoints_3d,
        keypoints_2d,
        camera_matrix,
        dist_coeffs
    )

    if success:
        # rvec: rotation vector, tvec: translation vector
        rotation_matrix, _ = cv2.Rodrigues(rvec)
        pose_6d = {
            'position': tvec.flatten(),
            'rotation': rotation_matrix
        }
        return pose_6d

    return None
```

---

### 6. Integração Visão + Reinforcement Learning

#### Observações Visuais em RL

Usar imagens como entrada para políticas de RL:

```python
import torch
import torch.nn as nn
from stable_baselines3 import PPO
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor

class CNNFeatureExtractor(BaseFeaturesExtractor):
    """
    CNN para extrair features de imagens para RL
    """
    def __init__(self, observation_space, features_dim=256):
        super().__init__(observation_space, features_dim)

        n_input_channels = observation_space.shape[0]

        self.cnn = nn.Sequential(
            nn.Conv2d(n_input_channels, 32, kernel_size=8, stride=4),
            nn.ReLU(),
            nn.Conv2d(32, 64, kernel_size=4, stride=2),
            nn.ReLU(),
            nn.Conv2d(64, 64, kernel_size=3, stride=1),
            nn.ReLU(),
            nn.Flatten(),
        )

        # Calcular dimensão de saída
        with torch.no_grad():
            sample = torch.zeros(1, *observation_space.shape)
            n_flatten = self.cnn(sample).shape[1]

        self.linear = nn.Sequential(
            nn.Linear(n_flatten, features_dim),
            nn.ReLU()
        )

    def forward(self, observations):
        return self.linear(self.cnn(observations))

# Usar com PPO
policy_kwargs = dict(
    features_extractor_class=CNNFeatureExtractor,
    features_extractor_kwargs=dict(features_dim=256)
)

model = PPO(
    "CnnPolicy",
    env,
    policy_kwargs=policy_kwargs,
    verbose=1
)
```

#### Ambiente com Observações Visuais

```python
import gymnasium as gym
from gymnasium import spaces
import numpy as np

class VisualPickEnv(gym.Env):
    """
    Ambiente de pick-and-place com observações visuais
    """
    def __init__(self, render_mode='rgb_array'):
        super().__init__()

        # Observação: imagem RGB 84x84x3
        self.observation_space = spaces.Box(
            low=0, high=255,
            shape=(3, 84, 84),
            dtype=np.uint8
        )

        # Ação: [dx, dy, dz, gripper]
        self.action_space = spaces.Box(
            low=-1, high=1,
            shape=(4,),
            dtype=np.float32
        )

    def reset(self, seed=None):
        # Setup simulação
        self.setup_scene()
        # Render primeira imagem
        obs = self.render_observation()
        return obs, {}

    def step(self, action):
        # Executar ação
        self.apply_action(action)
        # Render nova imagem
        obs = self.render_observation()
        reward = self.compute_reward()
        done = self.check_done()
        return obs, reward, done, False, {}

    def render_observation(self):
        """Renderiza câmera e retorna imagem processada"""
        # Capturar imagem da câmera simulada
        image = self.camera.get_image()  # (H, W, 3)

        # Resize para 84x84
        image = cv2.resize(image, (84, 84))

        # Normalizar e transpor para (C, H, W)
        image = image.transpose(2, 0, 1)

        return image
```

#### Domain Randomization para Sim2Real

**Problema**: Modelos treinados em simulação falham no mundo real.

**Solução**: Randomizar aparência visual na simulação para forçar robustez.

```python
import numpy as np
import cv2

class DomainRandomization:
    """
    Aplica randomização visual em imagens de treino
    """
    def __init__(self):
        pass

    def randomize(self, image):
        """
        Aplica múltiplas randomizações
        """
        image = self.randomize_brightness(image)
        image = self.randomize_contrast(image)
        image = self.randomize_hue(image)
        image = self.add_noise(image)
        image = self.randomize_blur(image)
        return image

    def randomize_brightness(self, image):
        """Varia brilho"""
        factor = np.random.uniform(0.5, 1.5)
        return np.clip(image * factor, 0, 255).astype(np.uint8)

    def randomize_contrast(self, image):
        """Varia contraste"""
        factor = np.random.uniform(0.5, 1.5)
        mean = np.mean(image)
        return np.clip((image - mean) * factor + mean, 0, 255).astype(np.uint8)

    def randomize_hue(self, image):
        """Varia matiz (cor)"""
        hsv = cv2.cvtColor(image, cv2.COLOR_RGB2HSV).astype(np.float32)
        hsv[:, :, 0] = (hsv[:, :, 0] + np.random.uniform(-10, 10)) % 180
        return cv2.cvtColor(hsv.astype(np.uint8), cv2.COLOR_HSV2RGB)

    def add_noise(self, image):
        """Adiciona ruído Gaussian"""
        noise = np.random.randn(*image.shape) * 10
        return np.clip(image + noise, 0, 255).astype(np.uint8)

    def randomize_blur(self, image):
        """Aplica blur aleatório"""
        if np.random.rand() > 0.5:
            ksize = np.random.choice([3, 5])
            return cv2.GaussianBlur(image, (ksize, ksize), 0)
        return image

# Integrar ao ambiente
class RandomizedVisualEnv(VisualPickEnv):
    def __init__(self):
        super().__init__()
        self.randomizer = DomainRandomization()

    def render_observation(self):
        image = super().render_observation()
        # Aplicar randomização durante treino
        if self.training:
            image = self.randomizer.randomize(image)
        return image
```

---

### 7. Integração com ROS 2

#### Nó ROS 2 para Detecção de Objetos

```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
from ultralytics import YOLO
import cv2

class YOLODetectorNode(Node):
    def __init__(self):
        super().__init__('yolo_detector')

        # Parâmetros
        self.declare_parameter('model_path', 'yolov8n.pt')
        self.declare_parameter('confidence_threshold', 0.5)

        model_path = self.get_parameter('model_path').value
        self.conf_thresh = self.get_parameter('confidence_threshold').value

        # Carregar modelo
        self.model = YOLO(model_path)
        self.bridge = CvBridge()

        # Subscribers
        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.image_callback,
            10
        )

        # Publishers
        self.detection_pub = self.create_publisher(
            Detection2DArray,
            '/detections',
            10
        )
        self.viz_pub = self.create_publisher(
            Image,
            '/detections/image',
            10
        )

        self.get_logger().info('YOLO Detector Node started')

    def image_callback(self, msg):
        # Converter ROS Image para OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Inferência
        results = self.model(cv_image, verbose=False)

        # Criar mensagem de detecções
        detections_msg = Detection2DArray()
        detections_msg.header = msg.header

        for box in results[0].boxes:
            if box.conf[0] < self.conf_thresh:
                continue

            detection = Detection2D()
            detection.header = msg.header

            # Bbox
            x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
            detection.bbox.center.x = float((x1 + x2) / 2)
            detection.bbox.center.y = float((y1 + y2) / 2)
            detection.bbox.size_x = float(x2 - x1)
            detection.bbox.size_y = float(y2 - y1)

            # Classe
            cls = int(box.cls[0])
            conf = float(box.conf[0])
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.id = str(cls)
            hypothesis.score = conf
            detection.results.append(hypothesis)

            detections_msg.detections.append(detection)

        # Publicar detecções
        self.detection_pub.publish(detections_msg)

        # Publicar visualização
        annotated = results[0].plot()
        viz_msg = self.bridge.cv2_to_imgmsg(annotated, encoding='bgr8')
        self.viz_pub.publish(viz_msg)

def main(args=None):
    rclpy.init(args=args)
    node = YOLODetectorNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## 💻 Prática Hands-On

### Projeto 1: Sistema de Detecção e Tracking

**Objetivo:** Criar pipeline completo de detecção → tracking → localização 3D.

```python
# visual_tracking_system.py
import cv2
import numpy as np
from ultralytics import YOLO

class VisualTrackingSystem:
    def __init__(self, camera_id=0):
        self.yolo = YOLO('yolov8n.pt')
        self.cap = cv2.VideoCapture(camera_id)
        self.tracker = None
        self.target_class = None

    def run(self):
        while True:
            ret, frame = self.cap.read()
            if not ret:
                break

            if self.tracker is None:
                # Modo detecção
                self.detect_and_select(frame)
            else:
                # Modo tracking
                self.track(frame)

            cv2.imshow('Visual Tracking', frame)
            key = cv2.waitKey(1) & 0xFF
            if key == ord('q'):
                break
            elif key == ord('r'):  # Reset
                self.tracker = None

        self.cap.release()
        cv2.destroyAllWindows()

    def detect_and_select(self, frame):
        # Detectar objetos
        results = self.yolo(frame)

        # Mostrar detecções
        annotated = results[0].plot()
        cv2.imshow('Visual Tracking', annotated)

        # Usuário clica em objeto para trackar
        # (implementar seleção por clique)

    def track(self, frame):
        success, bbox = self.tracker.update(frame)
        if success:
            x, y, w, h = [int(v) for v in bbox]
            cv2.rectangle(frame, (x, y), (x+w, y+h), (0, 255, 0), 2)

if __name__ == '__main__':
    system = VisualTrackingSystem()
    system.run()
```

---

### Projeto 2: Robô Pega Objeto Usando Visão

**Objetivo:** Usar YOLO + segmentação para pegar objeto específico.

[Código completo já fornecido nas seções anteriores - combinar detecção, segmentação e grasp planning]

---

### Projeto 3: Treinar Detector Custom

**Objetivo:** Criar dataset e treinar YOLOv8 para detectar objetos personalizados.

**Passo 1:** Coletar e anotar imagens

```bash
# Usar Roboflow para anotar
# https://roboflow.com/

# Ou usar LabelImg
pip install labelImg
labelImg
```

**Passo 2:** Treinar (ver código na seção YOLO)

**Passo 3:** Avaliar

```python
from ultralytics import YOLO

model = YOLO('runs/detect/train/weights/best.pt')
metrics = model.val()

print(f"mAP50: {metrics.box.map50:.3f}")
print(f"mAP50-95: {metrics.box.map:.3f}")
print(f"Precision: {metrics.box.mp:.3f}")
print(f"Recall: {metrics.box.mr:.3f}")
```

---

## 🏆 Projeto Final

### Sistema de Manipulação Visual Completo

**Objetivo:** Robô detecta, localiza, pega e coloca objetos usando apenas visão.

**Requisitos:**
1. ✅ Detecta 3+ classes de objetos (YOLO custom)
2. ✅ Localiza objetos em 3D (RGB-D)
3. ✅ Segmenta objeto para grasp preciso
4. ✅ Integra com controle de robô (ROS 2)
5. ✅ Taxa de sucesso > 80% em 10 tentativas

**Deliverables:**
- Código completo no GitHub
- Dataset de treinamento (se custom)
- Vídeo de demonstração
- Relatório técnico com métricas

---

## 📚 Recursos Adicionais

### Papers Importantes

1. **YOLO**: [You Only Look Once (Redmon et al., 2016)](https://arxiv.org/abs/1506.02640)
2. **YOLOv8**: [Ultralytics YOLOv8](https://github.com/ultralytics/ultralytics)
3. **Mask R-CNN**: [He et al., 2017](https://arxiv.org/abs/1703.06870)
4. **MediaPipe**: [Google MediaPipe](https://google.github.io/mediapipe/)

### Bibliotecas Essenciais

```bash
pip install opencv-python
pip install ultralytics
pip install mediapipe
pip install detectron2 -f https://dl.fbaipublicfiles.com/detectron2/wheels/cu118/torch2.0/index.html
pip install open3d  # Point cloud processing
```

### Datasets

- [COCO](https://cocodataset.org/) - 80 classes, 330K imagens
- [Open Images](https://storage.googleapis.com/openimages/web/index.html) - 600 classes, 9M imagens
- [Roboflow Universe](https://universe.roboflow.com/) - Datasets de robótica

---

## ✅ Checklist de Conclusão

- [ ] Calibrei uma câmera
- [ ] Implementei detecção com YOLO
- [ ] Treinei um modelo custom
- [ ] Realizei segmentação de instâncias
- [ ] Estimei poses humanas
- [ ] Converti 2D → 3D com precisão
- [ ] Integrei visão + RL
- [ ] Criei nó ROS 2 de visão
- [ ] Completei projeto final
- [ ] Publiquei código no GitHub

---

## 🚀 Próximos Passos

Você dominou visão computacional para robótica! Próximo módulo:

**Módulo 3.3**: Large Behavior Models (LBMs)

---

**Última atualização:** 2025-10-29
**Autor:** Programa FTH
**Licença:** MIT
