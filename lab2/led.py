import sys
import RPi.GPIO as GPIO
from time import sleep

LED_PIN = 23  # Пин светодиода

# Настройка GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(LED_PIN, GPIO.OUT)

def blink_led(pin, count, on_time=1.5, off_time=1.5):
    """
    Функция для мигания светодиода.
    :param pin: GPIO пин светодиода.
    :param count: Количество миганий.
    :param on_time: Время, в течение которого светодиод горит.
    :param off_time: Время, в течение которого светодиод не горит.
    """
    for i in range(count):
        GPIO.output(pin, True)  # Включить светодиод
        print(f"Мигание {i + 1}: LED включен.")
        sleep(on_time)
        GPIO.output(pin, False)  # Выключить светодиод
        print(f"Мигание {i + 1}: LED выключен.")
        sleep(off_time)

def main():
    try:
        print("Начало мигания светодиода.")
        blink_led(LED_PIN, count=10)
        print("Мигание завершено.")
    except KeyboardInterrupt:
        print("\nПрограмма прервана пользователем.")
    except Exception as e:
        print(f"Произошла ошибка: {e}")
    finally:
        GPIO.cleanup()
        print("GPIO очищен.")

if __name__ == "__main__":
    main()
