import cv2
import numpy as np
from gpiozero import Servo, LED
from gpiozero.pins.pigpio import PiGPIOFactory
from time import sleep
import threading
import RPi.GPIO as GPIO
from concurrent.futures import ThreadPoolExecutor

# Константы
IS_DEBUG = 0
LED_PIN = 4
SERVO_PIN = 14
VIDEO_DEVICE = 0

# Инициализация устройств
led = LED(LED_PIN)
led_state = False  # текущее состояние LED

if not IS_DEBUG:
    pigpio_factory = PiGPIOFactory()
    servo = Servo(SERVO_PIN, pin_factory=pigpio_factory)

cap = cv2.VideoCapture(VIDEO_DEVICE)

def led_control():
    """
    Функция для управления светодиодом в отдельном потоке.
    """
    global led_state
    led_state_old = False
    try:
        while True:
            sleep(0.1)  # Небольшая задержка для снижения нагрузки
            if led_state and not led_state_old:
                print("LED включен")
                led.on()
                led_state_old = True
            elif not led_state and led_state_old:
                print("LED выключен")
                led.off()
                led_state_old = False
    except KeyboardInterrupt:
        print("Поток управления LED завершен.")
    except Exception as e:
        print(f"Ошибка в led_control: {e}")

def camera_loop():
    """
    Основной цикл обработки камеры: детекция объектов и управление сервоприводом.
    """
    global led_state
    try:
        print("Запуск camera_loop")
        while True:
            ret, frame = cap.read()
            if not ret:
                print("Ошибка: кадр не получен.")
                break

            # Конвертация в пространство HSV
            hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

            # Диапазоны для детекции красного цвета
            lower1 = np.array([0, 100, 20])
            upper1 = np.array([10, 255, 255])
            lower2 = np.array([160, 100, 20])
            upper2 = np.array([179, 255, 255])

            # Маски для красного цвета
            lower_mask = cv2.inRange(hsv, lower1, upper1)
            upper_mask = cv2.inRange(hsv, lower2, upper2)
            mask = lower_mask + upper_mask

            # Устранение шума
            kernel = np.ones((5, 5), np.uint8)
            mask = cv2.morphologyEx(mask, cv2.MORPH_OPEN, kernel)
            mask = cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel)

            # Поиск кругов методом Хафа
            circles = cv2.HoughCircles(mask, cv2.HOUGH_GRADIENT, dp=1.5, minDist=100,
                                        param1=50, param2=30, minRadius=0, maxRadius=0)

            if circles is not None:
                led_state = True  # Обнаружен объект, включаем LED
                circles = np.uint16(np.around(circles))
                max_circle = max(circles[0], key=lambda x: x[2])
                x, y, r = max_circle

                center_x = x / frame.shape[1]
                center = (center_x - 0.5) * 2  # Приведение к значению [-1, 1]

                if not IS_DEBUG:
                    servo.value = center  # Управление сервоприводом

                # Отрисовка рамки вокруг объекта
                cv2.circle(frame, (x, y), r, (0, 255, 0), 2)
            else:
                led_state = False  # Объект не обнаружен, выключаем LED

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
        cap.release()
        cv2.destroyAllWindows()
        GPIO.cleanup()
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
