import sys
from hcsr04sensor import sensor
import RPi.GPIO as GPIO
from time import sleep

TRIG = 23  # Пин для сигнала "триггер"
ECHO = 24  # Пин для сигнала "эхо"

def measure_distance(trig_pin, echo_pin, attempts=5, delay=0.3):
    """
    Функция для измерения расстояния с использованием ультразвукового датчика.
    :param trig_pin: Пин для триггера.
    :param echo_pin: Пин для эхо.
    :param attempts: Количество измерений.
    :param delay: Задержка между измерениями.
    """
    try:
        GPIO.setmode(GPIO.BCM)
        measurements = []
        print("Начало измерений:")
        
        for i in range(attempts):
            # Создаем экземпляр измерения
            sensor_measurement = sensor.Measurement(trig_pin, echo_pin)
            raw_dist = sensor_measurement.raw_distance()
            distance_cm = sensor_measurement.distance_metric(raw_dist)
            
            measurements.append(distance_cm)
            print(f"Измерение {i + 1}: {distance_cm:.2f} см")
            sleep(delay)
        
        print("Измерения завершены.")
        return measurements
    except KeyboardInterrupt:
        print("\nПрограмма прервана пользователем.")
    except Exception as e:
        print(f"Произошла ошибка: {e}")
        return []
    finally:
        GPIO.cleanup()
        print("GPIO очищен.")

if __name__ == "__main__":
    distances = measure_distance(TRIG, ECHO, attempts=5, delay=0.3)
    print("Результаты:", distances)
