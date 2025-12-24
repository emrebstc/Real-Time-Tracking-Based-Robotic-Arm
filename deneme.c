#include "main.h"
#include <string.h>
#include <stdlib.h>
#include <stdio.h>
#include <math.h>
#include "deneme.h"

#define START_CHAR 'D'

extern UART_HandleTypeDef huart2;
extern UART_HandleTypeDef huart3;
extern TIM_HandleTypeDef htim2;
extern TIM_HandleTypeDef htim3;
extern uint8_t rx_buffer_it;
extern uint32_t Distance;

void Servo_SetAngle(uint8_t servoIndex, float angle);
uint32_t Measure_Distance(void);

uint8_t uartLineBuffer[RX_BUFFER_SIZE];
uint8_t uartLineIndex = 0;

volatile Coordinates sharedCoordinates = {0, 0};

static volatile int rx_state = 0;


int parseCoordinates(const char* data, Coordinates* coord)
{
    coord->dx = 0;
    coord->dy = 0;
    int items_read = 0;

    const char *dx_start = strstr(data, "DX:");
    if (dx_start)
    {
        dx_start += 3;
        const char *dx_end = strchr(dx_start, ',');
        if (dx_end)
        {
            char temp_dx[16];
            int len = dx_end - dx_start;
            if (len > 0 && len < 16) {
                strncpy(temp_dx, dx_start, len);
                temp_dx[len] = '\0';
                coord->dx = atoi(temp_dx);
                items_read++;
            }
        }
    }

    const char *dy_start = strstr(data, "DY:");
    if (dy_start)
    {
        dy_start += 3;
        const char *dy_end = strchr(dy_start, '\n');
        if (!dy_end) dy_end = strchr(dy_start, '\r');

        if (dy_end)
        {
            char temp_dy[16];
            int len = dy_end - dy_start;
            if (len > 0 && len < 16) {
                strncpy(temp_dy, dy_start, len);
                temp_dy[len] = '\0';
                coord->dy = atoi(temp_dy);
                items_read++;
            }
        }
    }

    return items_read;
}


void UpdateCoordinatesFromLine(const char* line)
{
    Coordinates tempCoord;
    int success = parseCoordinates(line, &tempCoord);
    if (success == 2)
    {
        sharedCoordinates = tempCoord;
    }
}


void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
    if (huart->Instance == USART2)
    {
        uint8_t rxByte = rx_buffer_it;

        if (rx_state == 0)
        {
            if (rxByte == START_CHAR)
            {
                uartLineIndex = 0;
                uartLineBuffer[uartLineIndex++] = rxByte;
                rx_state = 1;
            }
        }
        else
        {
            if (uartLineIndex < RX_BUFFER_SIZE - 1)
            {
                uartLineBuffer[uartLineIndex++] = rxByte;

                if (rxByte == '\n' || rxByte == '\r')
                {
                    uartLineBuffer[uartLineIndex] = '\0';
                    UpdateCoordinatesFromLine((const char*)uartLineBuffer);
                    rx_state = 0;
                }
            }
            else
            {
                rx_state = 0;
                uartLineIndex = 0;
            }
        }
        HAL_UART_Receive_IT(&huart2, &rx_buffer_it, 1);
    }
}

void ServoFollow6(void)
{
    static float current_angle = 90.0f;

    const int INPUT_MIN = 0;
    const int INPUT_MAX = 240;
    const float OUTPUT_MIN = 180.0f;
    const float OUTPUT_MAX = 0.0f;
    const int deadband = 5;

    int dx = sharedCoordinates.dx;
    int dx_error = dx - 120;
    float newAngle = 90.0f;

    if (abs(dx_error) > deadband)
    {
        float ratio = ((float)dx - INPUT_MIN) / (INPUT_MAX - INPUT_MIN);

        newAngle = OUTPUT_MIN + (ratio * (OUTPUT_MAX - OUTPUT_MIN));

        if (newAngle < 0) newAngle = 0;
        if (newAngle > 180) newAngle = 180;
    }
    else
    {
        newAngle = 90.0f;
    }

    const float STEP_SIZE = 0.1f;

    if (fabs(newAngle - current_angle) > 0.5f)
    {
        if (newAngle > current_angle) current_angle += STEP_SIZE;
        else current_angle -= STEP_SIZE;
        Servo_SetAngle(6, current_angle);
    }
    else if (abs(dx_error) <= deadband && fabs(90.0f - current_angle) > 0.2f)
    {
        if (90.0f > current_angle) current_angle += 0.2f;
        else current_angle -= 0.2f;
        Servo_SetAngle(6, current_angle);
    }
}


void ServoFollow3(void)
{
    static float current_angle = 135.0f;

    const int INPUT_MIN = -120;
    const int INPUT_MAX = 120;

    const float OUTPUT_MIN = 90.0f;

    const float OUTPUT_MAX = 180.0f;

    const int deadband = 5;

    int dy = sharedCoordinates.dy;
    int dy_error = dy - 0;

    float newAngle = 135.0f;

    if (abs(dy_error) > deadband)
    {

        float ratio = ((float)dy - INPUT_MIN) / (INPUT_MAX - INPUT_MIN);
        newAngle = OUTPUT_MIN + (ratio * (OUTPUT_MAX - OUTPUT_MIN));

        if (newAngle < 90) newAngle = 90;
        if (newAngle > 180) newAngle = 180;
    }
    else
    {
        newAngle = 135.0f;
    }

    const float STEP_SIZE = 1.5f;

    if (fabs(newAngle - current_angle) > 0.2f)
    {
        if (newAngle > current_angle) current_angle += STEP_SIZE;
        else current_angle -= STEP_SIZE;
        Servo_SetAngle(3, current_angle);
    }
    else if (abs(dy_error) <= deadband && fabs(135.0f - current_angle) > 0.2f)
    {
        if (135.0f > current_angle) current_angle += 0.2f;
        else current_angle -= 0.2f;
        Servo_SetAngle(3, current_angle);
    }
}


void ServoFollow5(void)
{
    static float current_angle_5 = 90.0f;

    static float current_angle_2 = 180.0f;

    uint32_t distance = Distance;

    const uint32_t INPUT_MIN_CM = 5;
    const uint32_t INPUT_MAX_CM = 15;

    const float OUTPUT_MIN_DEG = 90.0f;
    const float OUTPUT_MAX_DEG = 180.0f;

    const float STEP_SIZE = 0.5f;
    const float ANGLE_DEADBAND = 2.0f;

    float target_angle_5 = current_angle_5;

    if (distance >= INPUT_MAX_CM)
    {
        target_angle_5 = OUTPUT_MAX_DEG;
    }
    else if (distance <= INPUT_MIN_CM)
    {
        target_angle_5 = OUTPUT_MIN_DEG;
    }
    else
    {
        float ratio = ((float)distance - INPUT_MIN_CM) / (float)(INPUT_MAX_CM - INPUT_MIN_CM);
        target_angle_5 = OUTPUT_MIN_DEG + (ratio * (OUTPUT_MAX_DEG - OUTPUT_MIN_DEG));

        if (target_angle_5 < OUTPUT_MIN_DEG) target_angle_5 = OUTPUT_MIN_DEG;
        if (target_angle_5 > OUTPUT_MAX_DEG) target_angle_5 = OUTPUT_MAX_DEG;
    }

    if (fabs(target_angle_5 - current_angle_5) > ANGLE_DEADBAND)
    {
        if (target_angle_5 > current_angle_5)
            current_angle_5 += STEP_SIZE;
        else
            current_angle_5 -= STEP_SIZE;

        if (current_angle_5 < OUTPUT_MIN_DEG) current_angle_5 = OUTPUT_MIN_DEG;
        if (current_angle_5 > OUTPUT_MAX_DEG) current_angle_5 = OUTPUT_MAX_DEG;

        Servo_SetAngle(5, current_angle_5);

        float dev_5 = current_angle_5 - OUTPUT_MIN_DEG;

        float target_angle_2 = 180.0f - dev_5;

        current_angle_2 = target_angle_2;

        // Servo 2 Açıyı sınırla
        if (current_angle_2 < 90.0f) current_angle_2 = 90.0f;
        if (current_angle_2 > 180.0f) current_angle_2 = 180.0f;

        Servo_SetAngle(2, current_angle_2);
    }
}

void GripControl(void)
{
    const int GRIP_STABILITY_THRESHOLD = 50;

    int dx_error = abs(sharedCoordinates.dx - 120);
    int dy_error = abs(sharedCoordinates.dy - 0);

    const uint32_t GRIP_DISTANCE = 5;

    if (dx_error <= GRIP_STABILITY_THRESHOLD &&
        dy_error <= GRIP_STABILITY_THRESHOLD &&
        Distance <= GRIP_DISTANCE)
    {

        Servo_SetAngle(1, 89.0f);
    }
    else if (Distance > GRIP_DISTANCE + 2)
    {
        // Kıskaçı aç
        Servo_SetAngle(1, 45.0f);
    }
}


// Diğer görevler
void LoggerTask(void)
{
    char msg[100];

    while (1)
    {
        snprintf(msg, sizeof(msg), "Logger: DX=%d, DIST=%u cm\r\n", sharedCoordinates.dx, Distance);
        HAL_UART_Transmit(&huart3, (uint8_t*)msg, strlen(msg), HAL_MAX_DELAY);

        HAL_Delay(500);
    }
}

