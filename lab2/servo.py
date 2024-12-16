import sys
import RPi.GPIO as GPIO
from time import sleep

SERVO_PIN = 25  # Пин сервопривода

# Настройка GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(SERVO_PIN, GPIO.OUT)

# Инициализация PWM сигнала на пине сервопривода с частотой 50 Гц
pwm = GPIO.PWM(SERVO_PIN, 50)
pwm.start(0)

def set_angle(angle):
    """
    Устанавливает угол для сервопривода.
    :param angle: Угол (0-180 градусов).
    """
    try:
        if 0 <= angle <= 180:
            duty = angle / 18 + 2
            GPIO.output(SERVO_PIN, True)
            pwm.ChangeDutyCycle(duty)
            sleep(1)  # Время для достижения угла
            GPIO.output(SERVO_PIN, False)
            pwm.ChangeDutyCycle(0)
        else:
            print("Угол должен быть в пределах от 0 до 180 градусов.")
    except Exception as e:
        print(f"Ошибка установки угла: {e}")

def main():
    try:
        print("Установка углов для сервопривода.")
        print("Угол 0°")
        set_angle(0)
        sleep(0.5)

        print("Угол 45°")
        set_angle(45)
        sleep(0.5)

        print("Угол 90°")
        set_angle(90)
        sleep(0.5)

        print("Угол 180°")
        set_angle(180)
        sleep(0.5)

        print("Программа завершена.")
    except KeyboardInterrupt:
        print("\nПрограмма прервана пользователем.")
    except Exception as e:
        print(f"Произошла ошибка: {e}")
    finally:
        pwm.stop()
        GPIO.cleanup()
        print("GPIO очищен.")

if __name__ == "__main__":
    main()
