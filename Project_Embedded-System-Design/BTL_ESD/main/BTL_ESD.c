#include <stdio.h>
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "freertos/queue.h"
#include "esp_system.h"
#include "driver/gpio.h"
#include "driver/uart.h"
#include "esp_timer.h"
#include "rom/ets_sys.h"
#include <string.h>

#define DHT_PIN 19

#define KY026_PIN 23

#define FC37_PIN 27

#define PIR_PIN 14

#define SIM800_TX 3
#define SIM800_RX 1
#define BUF_SIZE (1024)

#define BUZZER_PIN 2

#define MOTOR_IN1 32
#define MOTOR_IN2 33

// Liệt kê các trạng thái sẽ có của DHT11
enum DHT11_Status 
{
    DHT11_CRC_ERROR = -2,      // Trạng thái dữ liệu truyền bị hỏng, các bit được truyền bị lỗi
    DHT11_TIMEOUT_ERROR,       // Trạng thái truyền bị quá thời gian tối đa cho phép
    DHT11_CORRECT_OPERATION    // Trạng thái truyền thành công
};

// Struct chứa các dữ liệu quan trọng mà ta quan tâm khi đọc DHT11
struct DHT11_Important_Parameters 
{
    int Status;
    int Temperature;
    int Humidity;
};

static int64_t Last_Read_Time = -10000000; 
static struct DHT11_Important_Parameters Last_Read; 

// Đếm thời gian mà chân DHT_PIN ở mức Level (0 hoặc 1). Nếu thời gian lớn hơn Time_Max thì kết luận lỗi TimeOut
// Time_Max: số micro giây tối đa có thể đếm để không bị lỗi 
// Level: mức logic mà ta sẽ đếm thời gian chân DHT_PIN duy trì mức đó
static int Counting_Time(uint16_t Time_Max, int Level) 
{
    int count = 0; 
    while (gpio_get_level(DHT_PIN) == Level) 
    { 
        if (count++ > Time_Max) 
            return DHT11_TIMEOUT_ERROR;
        ets_delay_us(1); // Mỗi lần lặp sẽ tương ứng với 1 lần delay 1 micro giây
    }
    return count; // Trả về số lần lặp đã xảy ra (số micro giây mà chân DHT11 đã ở mức Level)
}

// Kiểm tra tính toàn vẹn của dữ liệu DHT11 truyền đến (checksum)
static int Check_CRC_Error(uint8_t data[]) 
{
    if (data[4] == (data[0] + data[1] + data[2] + data[3]))
        return DHT11_CORRECT_OPERATION;
    else
        return DHT11_CRC_ERROR;
}

// Gửi yêu cầu đọc dữ liệu
static void MCU_Send_Start_Signal() 
{
    gpio_set_direction(DHT_PIN, GPIO_MODE_OUTPUT); // Chuyển về mode OUTPUT để xuất tín hiệu yêu cầu DHT11 đọc dữ liệu

    gpio_set_level(DHT_PIN, 0);
    ets_delay_us(20 * 1000); // Giữ mức thấp trong 20ms (20 * 1000 micro giây)

    gpio_set_level(DHT_PIN, 1);
    ets_delay_us(40); // Giữ mức cao trong 40 micro giây

    gpio_set_direction(DHT_PIN, GPIO_MODE_INPUT); // Chuyển về mode INPUT để đọc dữ liệu DHT11 gửi đến
}

// Đợi thông tin phản hồi lại (xác nhận truyền) từ DHT11
static int MCU_Check_Response() 
{
    // Đợi 80 micro giây ở mức thấp
    if (Counting_Time(80, 0) == DHT11_TIMEOUT_ERROR)
        return DHT11_TIMEOUT_ERROR;

    // Đợi 80 micro giây ở mức cao
    if (Counting_Time(80, 1) == DHT11_TIMEOUT_ERROR) 
        return DHT11_TIMEOUT_ERROR;

    return DHT11_CORRECT_OPERATION;
}

// Hàm _TimeOut_Error trả về kết quả khi DHT11 quá thời gian phản hồi
// Hàm trả về status: DHT11_TIMEOUT_ERROR (-1), Humidity -1, Temperature -1 để biểu thị trạng thái lỗi
static struct DHT11_Important_Parameters _TimeOut_Error() 
{
    struct DHT11_Important_Parameters TimeOut_Error = {DHT11_TIMEOUT_ERROR, -1, -1};
    return TimeOut_Error;
}

// Hàm _CRC_Error báo dữ liệu truyền bị hỏng, không toàn vẹn
// Hàm trả về status: DHT11_CRC_ERROR (-2), Humidity -1, Temperature -1 để biểu thị trạng thái lỗi
static struct DHT11_Important_Parameters _CRC_Error() 
{
    struct DHT11_Important_Parameters CRC_Error = {DHT11_CRC_ERROR, -1, -1};
    return CRC_Error;
}


// Gửi tín hiệu đọc, chờ phản hồi, đọc và xử lý 40 bit data nhận được từ DHT11
struct DHT11_Important_Parameters DHT11_Read() {
    // Nếu thời gian kể từ lần đọc cuối cùng chưa đủ 10s, ta trả về kết quả của lần đọc trước
    if (esp_timer_get_time() - 10000000 < Last_Read_Time) 
        return Last_Read;
    
    Last_Read_Time = esp_timer_get_time(); // Nếu đã đủ 10 giây, cập nhật thời gian kể từ lần đọc cuối cùng

    uint8_t data[5] = {0,0,0,0,0}; // Mảng lưu 40 bit dữ liệu DHT11 sẽ gửi tới

    MCU_Send_Start_Signal();

    if (MCU_Check_Response() == DHT11_TIMEOUT_ERROR)
        return Last_Read = _TimeOut_Error();
    
    // Bắt đầu đọc và xử lý thông tin từ DHT11
    for (int i = 0; i < 40; i++) 
    {
        // Nếu thời gian mức thấp mà DHT11 truyền đến (start transmit 1 bit data) lớn hơn 50 micro giây, ta kết luận lỗi TimeOut
        if (Counting_Time(50, 0) == DHT11_TIMEOUT_ERROR) 
            return Last_Read = _TimeOut_Error();

        // Nếu là bit 0 thì DHT11 sẽ duy trì mức 1 trong 28 micro giây
        // Nếu thời gian DHT11 duy trì ở mức 1 lớn hơn 28 micro giây thì DHT11 đã truyền đến bit 1           
        if (Counting_Time(70, 1) > 28) 
            data[i/8] |= (1 << (7-(i%8))); // Xử lí các bit 1 tương ứng với từng byte (bit 0 không cần xử lý vì mặc định đã là 0)
        
    }
    
    // Nếu không có lỗi hỏng dữ liệu và lỗi TimeOut ở trên, ta đọc được nhiệt độ là data[2] và độ ẩm là data[0]
    if (Check_CRC_Error(data) != DHT11_CRC_ERROR) 
    {
        Last_Read.Status = DHT11_CORRECT_OPERATION;
        Last_Read.Temperature = data[2];
        Last_Read.Humidity = data[0];
        return Last_Read;
    } 
    else  
    {
        return Last_Read = _CRC_Error();
    }
}

void sendATCommand(const char *command) 
{
    uart_write_bytes(UART_NUM_1, command, strlen(command));
}

void configureUART() 
{
    uart_config_t uart_config = {
        .baud_rate = 9600,
        .data_bits = UART_DATA_8_BITS,
        .parity    = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE
    };
    uart_param_config(UART_NUM_1, &uart_config);
    uart_set_pin(UART_NUM_1, SIM800_RX, SIM800_TX, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
    uart_driver_install(UART_NUM_1, BUF_SIZE * 2, 0, 0, NULL, 0);
}

void IRAM_ATTR button_isr_handler(void* arg) {
    uint32_t gpio_num = (uint32_t) arg;
    
    // Xử lý ngắt cho cảm biến KY_026 khi phát hiện có hồng ngoại ngọn lửa
    if (gpio_num == KY026_PIN) 
    {
        printf("KY_026 Status is 0\n");
        if (DHT11_Read().Temperature > 50)
        {
            // Đặt số điện thoại đích, gửi tin nhắn và rung còi báo động
            sendATCommand("AT+CMGS=\"+84852517099\"\r\n");
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            sendATCommand("Nha cua ban dang co chay!\x1A\r\n"); 
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            printf("Nha cua ban dang co chay!\n");
            gpio_set_level(BUZZER_PIN, 1);
            vTaskDelay(1000 / portTICK_PERIOD_MS);
            gpio_set_level(BUZZER_PIN, 0);
        }
    } 
    
    // Xử lý ngắt cho cảm biến FC-37 khi phát hiện có giọt nước rơi trên cảm biến
    else if (gpio_num == FC37_PIN) 
    {
        printf("FC-37 Status is 0\n");
        if (DHT11_Read().Humidity > 80)
        {
            // Motor kéo về, đóng cửa sổ
            gpio_set_level(MOTOR_IN1, 0);
            gpio_set_level(MOTOR_IN2, 1);
            vTaskDelay(5000 / portTICK_PERIOD_MS);
        }
    }

    // Xử lý ngắt cho trường hợp còn lại là cảm biến PIR khi phát hiện có người lạ đột nhập
    else 
    {
        printf("PIR Status is 1\n");
        // Đặt số điện thoại đích, gửi tin nhắn và rung còi báo động
        sendATCommand("AT+CMGS=\"+84852517099\"\r\n");
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        sendATCommand("Nha cua ban dang co trom!\x1A\r\n"); 
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        printf("Nha cua ban dang co trom!\n");
        gpio_set_level(BUZZER_PIN, 1);
        vTaskDelay(1000 / portTICK_PERIOD_MS);
        gpio_set_level(BUZZER_PIN, 0);
    }
}

void app_main(void)
{
    // Bắt đầu thiết lập I/O, UART
    configureUART();

    vTaskDelay(1000 / portTICK_PERIOD_MS);  

    sendATCommand("AT\r\n");
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    sendATCommand("AT+CMGF=1\r\n");  
    vTaskDelay(1000 / portTICK_PERIOD_MS);

    gpio_config_t Gpio_config = {}; 
    Gpio_config.pin_bit_mask = (1ULL << BUZZER_PIN) | (1ULL << MOTOR_IN1) | (1ULL << MOTOR_IN2);      
    Gpio_config.mode = GPIO_MODE_OUTPUT;           
    Gpio_config.pull_up_en = GPIO_PULLUP_DISABLE;      
    Gpio_config.pull_down_en = GPIO_PULLDOWN_DISABLE;   
    Gpio_config.intr_type = GPIO_INTR_DISABLE;  
    gpio_config(&Gpio_config); 

    Gpio_config.pin_bit_mask = (1ULL << KY026_PIN) | (1ULL << FC37_PIN) | (1ULL << PIR_PIN);      
    Gpio_config.mode = GPIO_MODE_INPUT;
    Gpio_config.pull_up_en = GPIO_PULLUP_DISABLE;
    Gpio_config.pull_down_en = GPIO_PULLDOWN_DISABLE;   
    Gpio_config.intr_type = GPIO_INTR_DISABLE; 
    gpio_config(&Gpio_config);

    // Ngắt ngoài cho cảm biến KY026, tích cực mức thấp
    Gpio_config.intr_type = GPIO_INTR_NEGEDGE;
    Gpio_config.pin_bit_mask = (1ULL << KY026_PIN);
    Gpio_config.mode = GPIO_MODE_INPUT;
    Gpio_config.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&Gpio_config);
    gpio_install_isr_service(0);
    gpio_isr_handler_add(KY026_PIN, button_isr_handler, (void*) KY026_PIN);

    // Ngắt ngoài cho cảm biến FC-37, tích cực mức thấp
    Gpio_config.intr_type = GPIO_INTR_NEGEDGE;
    Gpio_config.pin_bit_mask = (1ULL << FC37_PIN);
    Gpio_config.mode = GPIO_MODE_INPUT;
    Gpio_config.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&Gpio_config);
    gpio_isr_handler_add(FC37_PIN, button_isr_handler, (void*) FC37_PIN);

    // Ngắt ngoài cho cảm biến PIR, tích cực mức cao
    Gpio_config.intr_type = GPIO_INTR_POSEDGE;
    Gpio_config.pin_bit_mask = (1ULL << PIR_PIN);
    Gpio_config.mode = GPIO_MODE_INPUT;
    Gpio_config.pull_up_en = GPIO_PULLUP_DISABLE;
    gpio_config(&Gpio_config);
    gpio_isr_handler_add(PIR_PIN, button_isr_handler, (void*) PIR_PIN);

    uint8_t KY026_Status = 0;
    uint8_t FC37_Status = 0;
    uint8_t PIR_Status = 0;

    while (1)
{
    // Đọc nhiệt độ, độ ẩm từ DHT11
    printf("Temperature is %d \n", DHT11_Read().Temperature);
    printf("Humidity is %d\n", DHT11_Read().Humidity);
    printf("Status code is %d\n", DHT11_Read().Status);
    vTaskDelay(1000 / portTICK_PERIOD_MS);
    
    // Đọc trạng thái cảm biến KY_026
    KY026_Status = gpio_get_level(KY026_PIN);
    printf("KY_026 Status is %d\n", KY026_Status);

    // Đọc trạng thái cảm biến PIR
    PIR_Status = gpio_get_level(PIR_PIN);
    printf("PIR Status is %d\n", PIR_Status);
    
    // Đọc trạng thái cảm biến giọt nước FC-37
    FC37_Status = gpio_get_level(FC37_PIN);
    printf("FC37 Status is %d\n\n", FC37_Status);
    if (FC37_Status == 1 && DHT11_Read().Humidity < 60) 
    {
        // Ở trạng thái bình thường khi không có mưa, Motor sẽ ở trạng thái đẩy ra, giữ cửa sổ mở
        gpio_set_level(MOTOR_IN1, 1);
        gpio_set_level(MOTOR_IN2, 0);
        vTaskDelay(5000 / portTICK_PERIOD_MS);
    }
    }
}