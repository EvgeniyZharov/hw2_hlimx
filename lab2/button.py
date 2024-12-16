import sys
import RPi.GPIO as GPIO
from time import sleep

BUTTON_PIN = 5  # Пин кнопки

# Настройка GPIO
GPIO.setmode(GPIO.BCM)
GPIO.setup(BUTTON_PIN, GPIO.IN, pull_up_down=GPIO.PUD_UP)

def main():
    try:
        print("Нажмите кнопку 3 раза.")
        click_cnt = 0  # Счетчик нажатий
        while click_cnt < 3:
            input_value = GPIO.input(BUTTON_PIN)
            if not input_value:  # Если кнопка нажата
                click_cnt += 1
                print(f"Кнопка нажата {click_cnt} раз(а).")
                # Задержка, чтобы избежать "дребезга" контактов
                while not GPIO.input(BUTTON_PIN):
                    sleep(0.05)
            sleep(0.1)  # Небольшая задержка для снижения нагрузки на CPU
        print("Завершено.")
    except KeyboardInterrupt:
        print("\nПрограмма прервана пользователем.")
    except Exception as e:
        print(f"Произошла ошибка: {e}")
    finally:
        GPIO.cleanup()
        print("GPIO очищен.")

if __name__ == "__main__":
    main()
