---
layout: page
title: "Módulo 2.2: Sensores e Controle"
permalink: /niveis/nivel-2/modulo-2/
---

# Módulo 2.2: Sensores e Controle

**Duração estimada:** 10-12 horas
**Nível:** Intermediário
**Pré-requisito:** Módulo 2.1 concluído

---

## Objetivos de Aprendizado

Ao final deste módulo, você será capaz de:

- [ ] Compreender diferentes tipos de sensores robóticos
- [ ] Ler e processar dados de câmera, LIDAR e IMU
- [ ] Usar OpenCV para visão computacional básica
- [ ] Implementar controle PID
- [ ] Controlar motores (position vs velocity control)
- [ ] Criar comportamentos reativos complexos
- [ ] Integrar múltiplos sensores em um sistema

---

## Parte 1: Tipos de Sensores (2-3h)

### 1.1 Visão Geral de Sensores

Robôs humanoides usam diversos sensores para perceber o ambiente:

```
SENSORES DO ROBÔ HUMANOIDE
│
├── EXTEROCEPTIVOS (ambiente externo)
│   ├── Câmera RGB
│   ├── Câmera Depth (profundidade)
│   ├── LIDAR (laser scanner)
│   └── Microfone (áudio)
│
├── PROPRIOCEPTIVOS (estado interno)
│   ├── IMU (Inertial Measurement Unit)
│   ├── Encoders (posição das juntas)
│   ├── Force/Torque sensors
│   └── Temperatura
│
└── PROCESSADOS
    ├── Odometria (posição estimada)
    ├── SLAM (mapa + localização)
    └── Pose estimation
```

### 1.2 Câmera RGB

**Como funciona:** Captura imagens coloridas do ambiente, similar a uma câmera de celular.

**Dados retornados:** Array NumPy de pixels (altura × largura × 3 canais RGB)

**ROS 2 Message Type:** `sensor_msgs/Image`

#### Exemplo: Ler câmera no ROS 2

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class CameraReader(Node):
    """Lê imagens da câmera e exibe"""

    def __init__(self):
        super().__init__('camera_reader')

        # CvBridge converte ROS Image para OpenCV
        self.bridge = CvBridge()

        # Subscreve ao topic da câmera
        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.callback_image,
            10
        )

        self.get_logger().info('Camera reader iniciado!')

    def callback_image(self, msg):
        """
        Callback chamado quando nova imagem chega

        Args:
            msg: sensor_msgs/Image
        """
        try:
            # Converte ROS Image para OpenCV format (BGR)
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

            # Pega dimensões
            height, width, channels = cv_image.shape
            self.get_logger().info(f'Imagem recebida: {width}x{height}')

            # Exibe imagem
            cv2.imshow('Camera', cv_image)
            cv2.waitKey(1)

        except Exception as e:
            self.get_logger().error(f'Erro ao processar imagem: {e}')

def main(args=None):
    rclpy.init(args=args)
    node = CameraReader()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

**Instalação cv_bridge:**

```bash
sudo apt install ros-humble-cv-bridge python3-opencv
pip install opencv-python
```

### 1.3 LIDAR (Light Detection and Ranging)

**Como funciona:** Emite lasers e mede tempo de retorno para calcular distâncias.

**Dados retornados:** Array de distâncias em diferentes ângulos (ex: 360 leituras de 0° a 360°)

**ROS 2 Message Type:** `sensor_msgs/LaserScan`

#### Exemplo: Visualizar dados LIDAR

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation

class LidarVisualizer(Node):
    """Visualiza dados LIDAR em tempo real"""

    def __init__(self):
        super().__init__('lidar_visualizer')

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.callback_lidar,
            10
        )

        self.ranges = []
        self.angles = []

        # Setup plot
        self.fig, self.ax = plt.subplots(subplot_kw={'projection': 'polar'})
        self.line, = self.ax.plot([], [], 'b.')

        self.get_logger().info('LIDAR visualizer iniciado!')

    def callback_lidar(self, msg):
        """Processa dados LIDAR"""
        # Extrai distâncias
        self.ranges = np.array(msg.ranges)

        # Calcula ângulos
        num_readings = len(self.ranges)
        self.angles = np.linspace(msg.angle_min, msg.angle_max, num_readings)

        # Filtra leituras inválidas
        valid_indices = (self.ranges > msg.range_min) & (self.ranges < msg.range_max)
        self.ranges = self.ranges[valid_indices]
        self.angles = self.angles[valid_indices]

        # Atualiza plot
        self.line.set_data(self.angles, self.ranges)
        self.ax.set_ylim(0, msg.range_max)

        plt.pause(0.01)

def main(args=None):
    rclpy.init(args=args)
    node = LidarVisualizer()

    plt.show()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 1.4 IMU (Inertial Measurement Unit)

**Como funciona:** Combina acelerômetro, giroscópio e magnetômetro para medir orientação e movimento.

**Dados retornados:**
- Orientação (quaternion ou ângulos de Euler)
- Velocidade angular
- Aceleração linear

**ROS 2 Message Type:** `sensor_msgs/Imu`

#### Exemplo: Detectar queda usando IMU

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
import numpy as np

class DetectorQueda(Node):
    """Detecta se robô está caindo usando IMU"""

    def __init__(self):
        super().__init__('detector_queda')

        self.subscription = self.create_subscription(
            Imu,
            '/imu',
            self.callback_imu,
            10
        )

        self.limiar_queda = 45.0  # graus
        self.get_logger().info('Detector de queda iniciado!')

    def quaternion_para_euler(self, x, y, z, w):
        """
        Converte quaternion para ângulos de Euler

        Returns:
            (roll, pitch, yaw) em graus
        """
        # Roll (rotação em X)
        sinr_cosp = 2 * (w * x + y * z)
        cosr_cosp = 1 - 2 * (x * x + y * y)
        roll = np.arctan2(sinr_cosp, cosr_cosp)

        # Pitch (rotação em Y)
        sinp = 2 * (w * y - z * x)
        if abs(sinp) >= 1:
            pitch = np.copysign(np.pi / 2, sinp)
        else:
            pitch = np.arcsin(sinp)

        # Yaw (rotação em Z)
        siny_cosp = 2 * (w * z + x * y)
        cosy_cosp = 1 - 2 * (y * y + z * z)
        yaw = np.arctan2(siny_cosp, cosy_cosp)

        # Converte radianos para graus
        return np.degrees(roll), np.degrees(pitch), np.degrees(yaw)

    def callback_imu(self, msg):
        """Processa dados IMU"""
        # Extrai quaternion
        x = msg.orientation.x
        y = msg.orientation.y
        z = msg.orientation.z
        w = msg.orientation.w

        # Converte para Euler
        roll, pitch, yaw = self.quaternion_para_euler(x, y, z, w)

        # Detecta queda
        if abs(roll) > self.limiar_queda or abs(pitch) > self.limiar_queda:
            self.get_logger().warn(f'ALERTA DE QUEDA! Roll={roll:.1f}°, Pitch={pitch:.1f}°')
        else:
            self.get_logger().info(f'Estável - Roll={roll:.1f}°, Pitch={pitch:.1f}°, Yaw={yaw:.1f}°')

def main(args=None):
    rclpy.init(args=args)
    node = DetectorQueda()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 1.5 Encoders (Posição das Juntas)

**Como funciona:** Medem rotação/posição de cada junta do robô.

**Dados retornados:** Ângulo ou posição linear de cada motor/junta.

**ROS 2 Message Type:** `sensor_msgs/JointState`

#### Exemplo: Monitorar estado das juntas

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState

class MonitorJuntas(Node):
    """Monitora estado de todas as juntas do robô"""

    def __init__(self):
        super().__init__('monitor_juntas')

        self.subscription = self.create_subscription(
            JointState,
            '/joint_states',
            self.callback_juntas,
            10
        )

        self.get_logger().info('Monitor de juntas iniciado!')

    def callback_juntas(self, msg):
        """Processa estado das juntas"""
        # JointState contém 3 arrays: name, position, velocity, effort

        self.get_logger().info('=== Estado das Juntas ===')

        for i, name in enumerate(msg.name):
            position = msg.position[i] if i < len(msg.position) else 0.0
            velocity = msg.velocity[i] if i < len(msg.velocity) else 0.0

            self.get_logger().info(
                f'{name}: pos={position:.2f} rad, vel={velocity:.2f} rad/s'
            )

def main(args=None):
    rclpy.init(args=args)
    node = MonitorJuntas()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Parte 2: Visão Computacional com OpenCV (3-4h)

### 2.1 Detecção de Cores

Use OpenCV para detectar objetos por cor:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2
import numpy as np

class DetectorCor(Node):
    """Detecta objetos de cor específica na imagem"""

    def __init__(self):
        super().__init__('detector_cor')

        self.bridge = CvBridge()

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.callback_image,
            10
        )

        self.get_logger().info('Detector de cor iniciado!')

    def callback_image(self, msg):
        """Detecta objetos vermelhos"""
        # Converte para OpenCV
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Converte BGR para HSV (melhor para detecção de cor)
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Define range de vermelho em HSV
        # Vermelho tem dois ranges (wraps around 0/180)
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])

        # Cria máscaras
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = mask1 | mask2

        # Encontra contornos
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        # Desenha contornos na imagem original
        for contour in contours:
            area = cv2.contourArea(contour)

            if area > 500:  # Filtra objetos pequenos (ruído)
                # Desenha contorno
                cv2.drawContours(cv_image, [contour], -1, (0, 255, 0), 3)

                # Calcula centro
                M = cv2.moments(contour)
                if M['m00'] != 0:
                    cx = int(M['m10'] / M['m00'])
                    cy = int(M['m01'] / M['m00'])

                    # Desenha centro
                    cv2.circle(cv_image, (cx, cy), 7, (255, 0, 0), -1)
                    cv2.putText(cv_image, f'Centro: ({cx}, {cy})',
                                (cx - 50, cy - 20),
                                cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 2)

                    self.get_logger().info(f'Objeto vermelho detectado em ({cx}, {cy}), área={area}')

        # Exibe imagens
        cv2.imshow('Original', cv_image)
        cv2.imshow('Mascara', mask)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = DetectorCor()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2.2 Detecção de Faces

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
import cv2

class DetectorFace(Node):
    """Detecta faces humanas usando Haar Cascade"""

    def __init__(self):
        super().__init__('detector_face')

        self.bridge = CvBridge()

        # Carrega classificador Haar Cascade pré-treinado
        cascade_path = cv2.data.haarcascades + 'haarcascade_frontalface_default.xml'
        self.face_cascade = cv2.CascadeClassifier(cascade_path)

        self.subscription = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.callback_image,
            10
        )

        self.get_logger().info('Detector de faces iniciado!')

    def callback_image(self, msg):
        """Detecta faces na imagem"""
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Converte para escala de cinza (requerido pelo Haar Cascade)
        gray = cv2.cvtColor(cv_image, cv2.COLOR_BGR2GRAY)

        # Detecta faces
        faces = self.face_cascade.detectMultiScale(
            gray,
            scaleFactor=1.1,
            minNeighbors=5,
            minSize=(30, 30)
        )

        # Desenha retângulos ao redor das faces
        for (x, y, w, h) in faces:
            cv2.rectangle(cv_image, (x, y), (x+w, y+h), (0, 255, 0), 2)
            cv2.putText(cv_image, 'Face',
                        (x, y-10),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.9, (0, 255, 0), 2)

            self.get_logger().info(f'Face detectada em ({x}, {y}), tamanho {w}x{h}')

        # Exibe
        cv2.imshow('Detector de Faces', cv_image)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = DetectorFace()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 2.3 Linha Follower (Seguidor de Linha)

Aplicação prática: robô que segue uma linha no chão.

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import cv2
import numpy as np

class SeguidorLinha(Node):
    """Robô que segue linha preta no chão"""

    def __init__(self):
        super().__init__('seguidor_linha')

        self.bridge = CvBridge()

        self.image_sub = self.create_subscription(
            Image,
            '/camera/image_raw',
            self.callback_image,
            10
        )

        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        self.get_logger().info('Seguidor de linha iniciado!')

    def callback_image(self, msg):
        """Processa imagem e controla robô"""
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')

        # Pega apenas região inferior da imagem (chão)
        height, width = cv_image.shape[:2]
        roi = cv_image[int(height*0.7):height, 0:width]

        # Converte para escala de cinza
        gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)

        # Threshold para binarizar (linha preta em fundo branco)
        _, binary = cv2.threshold(gray, 60, 255, cv2.THRESH_BINARY_INV)

        # Encontra momentos da imagem
        M = cv2.moments(binary)

        if M['m00'] > 0:
            # Calcula centroid da linha
            cx = int(M['m10'] / M['m00'])
            cy = int(M['m01'] / M['m00'])

            # Desenha centroid
            cv2.circle(roi, (cx, cy), 10, (0, 0, 255), -1)

            # Calcula erro (centro da imagem - centro da linha)
            erro = cx - width // 2

            # Controle proporcional
            kp = 0.005  # Ganho proporcional

            msg_vel = Twist()
            msg_vel.linear.x = 0.2  # Velocidade constante
            msg_vel.angular.z = -kp * erro  # Corrige direção

            self.cmd_vel_pub.publish(msg_vel)

            self.get_logger().info(f'Centro linha: {cx}, Erro: {erro}, Angular: {msg_vel.angular.z:.3f}')

        else:
            # Linha não detectada - para
            msg_vel = Twist()
            self.cmd_vel_pub.publish(msg_vel)
            self.get_logger().warn('Linha não detectada!')

        # Exibe
        cv2.imshow('ROI', roi)
        cv2.imshow('Binary', binary)
        cv2.waitKey(1)

def main(args=None):
    rclpy.init(args=args)
    node = SeguidorLinha()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Parte 3: Controle PID (3-4h)

### 3.1 O que é Controle PID?

PID (Proportional-Integral-Derivative) é um algoritmo de controle que minimiza erro entre valor desejado e valor atual.

```
        Setpoint (desejado)
              │
              ▼
         ┌────────┐
    ─────┤  PID   ├───────> Atuador (motor)
    │    └────────┘
    │         ▲
    │         │
    └─────────┘
       Feedback (sensor)
```

**Componentes:**

1. **P (Proporcional):** Reage proporcionalmente ao erro atual
   - `Kp * erro`
   - Maior Kp = resposta mais rápida, mas pode oscilar

2. **I (Integral):** Acumula erro ao longo do tempo
   - `Ki * soma_erros`
   - Elimina erro residual constante (drift)

3. **D (Derivativo):** Reage à taxa de mudança do erro
   - `Kd * (erro_atual - erro_anterior)`
   - Reduz overshoot e oscilações

### 3.2 Implementação PID em Python

```python
import time

class ControladorPID:
    """
    Controlador PID genérico

    Args:
        kp: Ganho proporcional
        ki: Ganho integral
        kd: Ganho derivativo
        setpoint: Valor desejado
    """

    def __init__(self, kp, ki, kd, setpoint=0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint

        self.erro_anterior = 0
        self.integral = 0
        self.tempo_anterior = time.time()

    def atualizar(self, valor_atual):
        """
        Calcula saída do controlador PID

        Args:
            valor_atual: Medição atual do sensor

        Returns:
            Saída de controle
        """
        # Calcula erro
        erro = self.setpoint - valor_atual

        # Calcula delta tempo
        tempo_atual = time.time()
        dt = tempo_atual - self.tempo_anterior

        if dt <= 0:
            dt = 0.01  # Evita divisão por zero

        # Termo Proporcional
        P = self.kp * erro

        # Termo Integral (acumula erro)
        self.integral += erro * dt
        I = self.ki * self.integral

        # Termo Derivativo (taxa de mudança)
        derivativo = (erro - self.erro_anterior) / dt
        D = self.kd * derivativo

        # Saída total
        output = P + I + D

        # Atualiza variáveis para próxima iteração
        self.erro_anterior = erro
        self.tempo_anterior = tempo_atual

        return output

    def reset(self):
        """Reseta integral e erro"""
        self.integral = 0
        self.erro_anterior = 0

# Exemplo de uso
if __name__ == '__main__':
    # Controlar temperatura de 25°C
    pid = ControladorPID(kp=2.0, ki=0.5, kd=0.1, setpoint=25.0)

    # Simula 10 leituras
    temperaturas = [20, 22, 23.5, 24.5, 25.2, 25.5, 25.3, 25.1, 25.0, 25.0]

    for temp in temperaturas:
        controle = pid.atualizar(temp)
        print(f'Temp: {temp}°C, Controle: {controle:.2f}')
        time.sleep(0.1)
```

### 3.3 PID para Controle de Posição

Aplicação: controlar posição de uma junta do robô.

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import JointState
from std_msgs.msg import Float64
import time

class ControladorPID:
    """Controlador PID (mesmo código anterior)"""
    def __init__(self, kp, ki, kd, setpoint=0):
        self.kp = kp
        self.ki = ki
        self.kd = kd
        self.setpoint = setpoint
        self.erro_anterior = 0
        self.integral = 0
        self.tempo_anterior = time.time()

    def atualizar(self, valor_atual):
        erro = self.setpoint - valor_atual
        tempo_atual = time.time()
        dt = tempo_atual - self.tempo_anterior
        if dt <= 0:
            dt = 0.01

        P = self.kp * erro
        self.integral += erro * dt
        I = self.ki * self.integral
        derivativo = (erro - self.erro_anterior) / dt
        D = self.kd * derivativo

        output = P + I + D

        self.erro_anterior = erro
        self.tempo_anterior = tempo_atual

        return output

class ControlePosicaoJunta(Node):
    """Controla posição de junta usando PID"""

    def __init__(self):
        super().__init__('controle_posicao_junta')

        # Configura PID
        self.pid = ControladorPID(kp=5.0, ki=0.1, kd=0.5, setpoint=1.57)  # 90 graus

        # Subscreve ao estado da junta
        self.joint_sub = self.create_subscription(
            JointState,
            '/joint_states',
            self.callback_joint_state,
            10
        )

        # Publica comando de velocidade
        self.cmd_pub = self.create_publisher(Float64, '/joint_velocity_controller/command', 10)

        self.get_logger().info('Controlador PID iniciado! Setpoint: 1.57 rad (90°)')

    def callback_joint_state(self, msg):
        """Lê posição atual e aplica controle PID"""
        # Assume primeira junta (índice 0)
        if len(msg.position) > 0:
            posicao_atual = msg.position[0]

            # Calcula saída PID
            velocidade_cmd = self.pid.atualizar(posicao_atual)

            # Limita velocidade
            velocidade_cmd = max(min(velocidade_cmd, 2.0), -2.0)

            # Publica comando
            msg_cmd = Float64()
            msg_cmd.data = velocidade_cmd
            self.cmd_pub.publish(msg_cmd)

            erro = self.pid.setpoint - posicao_atual
            self.get_logger().info(
                f'Pos: {posicao_atual:.3f} rad, Erro: {erro:.3f}, Cmd: {velocidade_cmd:.3f}'
            )

def main(args=None):
    rclpy.init(args=args)
    node = ControlePosicaoJunta()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

### 3.4 Tunning PID (Ajuste de Ganhos)

**Método Ziegler-Nichols (simplificado):**

1. **Comece com Ki=0 e Kd=0**
2. **Aumente Kp até o sistema oscilar constantemente**
3. **Anote Kp crítico (Ku) e período de oscilação (Tu)**
4. **Calcule:**
   - `Kp = 0.6 * Ku`
   - `Ki = 2 * Kp / Tu`
   - `Kd = Kp * Tu / 8`

**Regras práticas:**

- Sistema lento → aumenta Kp
- Erro residual → aumenta Ki
- Oscilações → aumenta Kd
- Sempre ajuste um ganho de cada vez!

---

## Parte 4: Integração Multi-Sensorial (2h)

### 4.1 Fusão de Sensores

Combine múltiplos sensores para navegação robusta:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan, Imu, Image
from geometry_msgs.msg import Twist
from cv_bridge import CvBridge
import numpy as np

class NavegacaoIntegrada(Node):
    """Navegação usando LIDAR + IMU + Câmera"""

    def __init__(self):
        super().__init__('navegacao_integrada')

        self.bridge = CvBridge()

        # Subscribers
        self.lidar_sub = self.create_subscription(LaserScan, '/scan', self.callback_lidar, 10)
        self.imu_sub = self.create_subscription(Imu, '/imu', self.callback_imu, 10)
        self.camera_sub = self.create_subscription(Image, '/camera/image_raw', self.callback_camera, 10)

        # Publisher
        self.cmd_vel_pub = self.create_publisher(Twist, '/cmd_vel', 10)

        # Timer para controle
        self.timer = self.create_timer(0.1, self.controlar)

        # Estado dos sensores
        self.obstaculo_proximo = False
        self.dist_minima = float('inf')
        self.inclinacao_perigosa = False
        self.objeto_vermelho_detectado = False
        self.objeto_vermelho_cx = 0

        self.get_logger().info('Navegação integrada iniciada!')

    def callback_lidar(self, msg):
        """Processa LIDAR"""
        ranges = np.array(msg.ranges)
        ranges_validos = ranges[(ranges > 0) & (ranges < msg.range_max)]

        if len(ranges_validos) > 0:
            self.dist_minima = np.min(ranges_validos)
            self.obstaculo_proximo = self.dist_minima < 0.8  # 80cm

    def callback_imu(self, msg):
        """Processa IMU"""
        # Converte quaternion para Euler (código simplificado)
        x, y, z, w = msg.orientation.x, msg.orientation.y, msg.orientation.z, msg.orientation.w
        roll = np.arctan2(2*(w*x + y*z), 1 - 2*(x*x + y*y))
        pitch = np.arcsin(2*(w*y - z*x))

        roll_graus = np.degrees(roll)
        pitch_graus = np.degrees(pitch)

        # Detecta inclinação perigosa
        self.inclinacao_perigosa = abs(roll_graus) > 30 or abs(pitch_graus) > 30

    def callback_camera(self, msg):
        """Detecta objetos vermelhos (simplificado)"""
        import cv2

        cv_image = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        hsv = cv2.cvtColor(cv_image, cv2.COLOR_BGR2HSV)

        # Máscara vermelha
        lower = np.array([0, 100, 100])
        upper = np.array([10, 255, 255])
        mask = cv2.inRange(hsv, lower, upper)

        # Encontra contornos
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        if contours:
            # Maior contorno
            largest = max(contours, key=cv2.contourArea)
            area = cv2.contourArea(largest)

            if area > 500:
                M = cv2.moments(largest)
                if M['m00'] > 0:
                    self.objeto_vermelho_cx = int(M['m10'] / M['m00'])
                    self.objeto_vermelho_detectado = True
                    return

        self.objeto_vermelho_detectado = False

    def controlar(self):
        """Lógica de controle integrada"""
        msg = Twist()

        # PRIORIDADE 1: Segurança - inclina perigosa
        if self.inclinacao_perigosa:
            msg.linear.x = 0.0
            msg.angular.z = 0.0
            self.get_logger().error('INCLINAÇÃO PERIGOSA - PARANDO!')

        # PRIORIDADE 2: Evitar obstáculo
        elif self.obstaculo_proximo:
            msg.linear.x = 0.0
            msg.angular.z = 0.5  # Gira
            self.get_logger().warn(f'Obstáculo a {self.dist_minima:.2f}m - Desviando')

        # PRIORIDADE 3: Seguir objeto vermelho
        elif self.objeto_vermelho_detectado:
            # Calcula erro (centro da imagem = 320 para câmera 640px)
            erro = self.objeto_vermelho_cx - 320
            msg.linear.x = 0.2
            msg.angular.z = -0.003 * erro  # Controle proporcional
            self.get_logger().info(f'Seguindo objeto vermelho, erro={erro}')

        # PRIORIDADE 4: Navegação livre
        else:
            msg.linear.x = 0.4
            msg.angular.z = 0.0
            self.get_logger().info('Navegação livre')

        self.cmd_vel_pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = NavegacaoIntegrada()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
```

---

## Mini-Projeto: Robô Reativo Completo

**Objetivo:** Criar robô que:
1. Navega evitando obstáculos (LIDAR)
2. Detecta e segue objetos vermelhos (câmera)
3. Para se detectar queda (IMU)
4. Usa PID para suavizar movimentos

**Checklist:**
- [ ] Implementar fusão LIDAR + câmera + IMU
- [ ] Controle PID para velocidade
- [ ] Sistema de prioridades de comportamento
- [ ] Logging e visualização
- [ ] Testar em simulador Webots

---

## Recursos Adicionais

### Documentação
- [OpenCV Python Docs](https://docs.opencv.org/4.x/d6/d00/tutorial_py_root.html)
- [ROS 2 Sensors](https://docs.ros.org/en/humble/p/sensor_msgs/)
- [PID Control Theory](https://en.wikipedia.org/wiki/PID_controller)

### Tutoriais
- [OpenCV Tutorial - PyImageSearch](https://pyimagesearch.com/)
- [PID Without a PhD](http://www.wescottdesign.com/articles/pid/pidWithoutAPhd.pdf)

---

## Troubleshooting

### Erro: cv_bridge não encontrado

```bash
sudo apt install ros-humble-cv-bridge
```

### Câmera não mostra imagem

Verifique se topic está publicando:
```bash
ros2 topic list
ros2 topic echo /camera/image_raw
```

### PID oscila muito

- Reduzir Kp
- Aumentar Kd
- Verificar se dt está calculado corretamente

---

## Checklist de Conclusão

- [ ] Ler dados de câmera RGB
- [ ] Processar dados LIDAR
- [ ] Usar IMU para orientação
- [ ] Implementar detecção de cor
- [ ] Implementar detecção de faces
- [ ] Criar seguidor de linha
- [ ] Implementar controlador PID
- [ ] Fazer tunning PID
- [ ] Integrar múltiplos sensores
- [ ] Mini-projeto "Robô Reativo" concluído

---

## Próximo Módulo

No **Módulo 2.3: Introdução a Aprendizado por Reforço**, você vai:
- Entender conceitos de RL
- Usar Stable-Baselines3
- Treinar robô com PPO
- Fazer robô aprender comportamentos

[→ Ir para Módulo 2.3]({{ '/niveis/nivel-2/modulo-3' | relative_url }}){: .btn .btn-primary}

---

**Última atualização:** 2025-10-29
