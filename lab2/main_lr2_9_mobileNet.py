import cv2
import os
import requests
import numpy as np
from time import sleep
from concurrent.futures import ThreadPoolExecutor

# Константы
IS_DEBUG = 0  # Если 1, работа с GPIO отключается
IS_RASPBERRY_PI = True  # Если False, используется Orange Pi
LED_PIN = 4
SERVO_PIN = 14
VIDEO_DEVICE = 0
CONF_THRESHOLD = 0.6  # Порог уверенности

# Подключение GPIO только если не DEBUG
if not IS_DEBUG:
    if IS_RASPBERRY_PI:
        from gpiozero import Servo, LED
        from gpiozero.pins.pigpio import PiGPIOFactory
        import RPi.GPIO as GPIO
        pigpio_factory = PiGPIOFactory()
        led = LED(LED_PIN)
        servo = Servo(SERVO_PIN, pin_factory=pigpio_factory)
    else:
        from pyA20.gpio import gpio
        from pyA20.gpio import port
        gpio.init()
        gpio.setcfg(LED_PIN, gpio.OUTPUT)
        gpio.setcfg(SERVO_PIN, gpio.PWM)
        led = None  # На Orange Pi управление LED вручную через gpio

# Пути к файлам моделей
MODEL_CONFIG = 'ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt'
MODEL_WEIGHTS = 'frozen_inference_graph.pb'
CLASS_FILE = 'coco.names'

# Ссылки на файлы моделей
MODEL_CONFIG_URL = 'https://raw.githubusercontent.com/zafarRehan/object_detection_COCO/refs/heads/main/ssd_mobilenet_v3_large_coco_2020_01_14.pbtxt'
MODEL_WEIGHTS_URL = 'http://download.tensorflow.org/models/object_detection/ssd_mobilenet_v3_large_coco_2020_01_14.tar.gz'
CLASS_FILE_URL = 'https://raw.githubusercontent.com/amikelive/coco-labels/master/coco-labels-paper.txt'

# Проверка и загрузка моделей
def download_file(url, file_path):
    """
    Скачивает файл, если он не существует.
    """
    if not os.path.exists(file_path):
        print(f"Загрузка {file_path}...")
        response = requests.get(url, stream=True)
        with open(file_path, 'wb') as f:
            for chunk in response.iter_content(chunk_size=1024):
                f.write(chunk)
        print(f"{file_path} загружен.")

# Распаковка архива весов модели
def extract_weights(archive_path, output_path):
    """
    Распаковывает архив модели.
    """
    import tarfile
    if not os.path.exists(output_path):
        print(f"Распаковка {archive_path}...")
        with tarfile.open(archive_path, 'r:gz') as tar:
            tar.extractall(path=os.path.dirname(output_path))
        print(f"{output_path} готово.")

# Скачивание и подготовка файлов моделей
def prepare_models():
    download_file(MODEL_CONFIG_URL, MODEL_CONFIG)
    weights_archive = 'ssd_mobilenet_v3_large_coco_2020_01_14.tar.gz'
    download_file(MODEL_WEIGHTS_URL, weights_archive)
    extract_weights(weights_archive, MODEL_WEIGHTS)
    download_file(CLASS_FILE_URL, CLASS_FILE)

# Подготовка моделей
prepare_models()

# Загрузка модели MobileNet
net = cv2.dnn_DetectionModel(MODEL_WEIGHTS, MODEL_CONFIG)
net.setInputSize(320, 320)
net.setInputScale(1.0 / 127.5)
net.setInputMean((127.5, 127.5, 127.5))
net.setInputSwapRB(True)

# Загрузка имен классов
with open(CLASS_FILE, 'r') as f:
    class_names = f.read().strip().split('\n')

def led_control():
    """
    Управление светодиодом в отдельном потоке.
    """
    if IS_DEBUG:
        print("Работа с GPIO отключена (DEBUG режим).")
        return

    led_state = False
    led_state_old = False
    try:
        while True:
            sleep(0.1)  # Небольшая задержка для снижения нагрузки
            if IS_RASPBERRY_PI:
                # Raspberry Pi
                if led_state and not led_state_old:
                    print("LED включен")
                    led.on()
                    led_state_old = True
                elif not led_state and led_state_old:
                    print("LED выключен")
                    led.off()
                    led_state_old = False
            else:
                # Orange Pi
                if led_state and not led_state_old:
                    print("LED включен")
                    gpio.output(LED_PIN, gpio.HIGH)
                    led_state_old = True
                elif not led_state and led_state_old:
                    print("LED выключен")
                    gpio.output(LED_PIN, gpio.LOW)
                    led_state_old = False
    except KeyboardInterrupt:
        print("Поток управления LED завершен.")
    except Exception as e:
        print(f"Ошибка в led_control: {e}")

def camera_loop():
    """
    Основной цикл обработки камеры: детекция объектов и управление сервоприводом.
    """
    try:
        print("Запуск camera_loop")
        cap = cv2.VideoCapture(VIDEO_DEVICE)
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Ошибка: кадр не получен.")
                break

            # Детекция объектов с помощью MobileNet
            class_ids, confs, bboxes = net.detect(frame, confThreshold=CONF_THRESHOLD)

            if len(class_ids) > 0:
                for class_id, conf, bbox in zip(class_ids.flatten(), confs.flatten(), bboxes):
                    if class_id <= len(class_names):  # Проверка валидности класса
                        x, y, w, h = bbox

                        center_x = (x + w / 2) / frame.shape[1]
                        center = (center_x - 0.5) * 2  # Приведение к значению [-1, 1]

                        if not IS_DEBUG and IS_RASPBERRY_PI:
                            servo.value = center  # Управление сервоприводом
                        elif not IS_DEBUG and not IS_RASPBERRY_PI:
                            gpio.output(SERVO_PIN, int(center * 100))

                        # Отрисовка рамки вокруг объекта
                        cv2.rectangle(frame, (x, y), (x + w, y + h), (0, 255, 0), 2)
                        cv2.putText(frame, f"{class_names[class_id - 1]} {conf:.2f}",
                                    (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

            cv2.imshow('frame', frame)

            # Выход по клавише 'q'
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

            sleep(0.1)  # Задержка для снижения нагрузки
    except KeyboardInterrupt:
        print("Поток обработки камеры завершен.")
    except Exception as e:
        print(f"Ошибка в camera_loop: {e}")
    finally:
        if not IS_DEBUG:
            cap.release()
        cv2.destroyAllWindows()
        print("Ресурсы камеры освобождены.")

# Запуск потоков
if __name__ == "__main__":
    try:
        with ThreadPoolExecutor(max_workers=2) as executor:
            executor.submit(led_control)
            executor.submit(camera_loop)
    except Exception as e:
        print(f"Основной поток завершился с ошибкой: {e}")
    finally:
        print("Программа завершена.")
