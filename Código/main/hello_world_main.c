#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <math.h>
#include <stdbool.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/queue.h"
#include "driver/i2c.h"
#include "driver/gpio.h"
#include "driver/ledc.h"
#include "esp_log.h"
#include "esp_vfs_fat.h"
#include "sdmmc_cmd.h"
#include "driver/sdmmc_host.h"
#include "mpu6050.h"
#include "ssd1306.h"

#define I2C_MASTER_SCL_IO           9     
#define I2C_MASTER_SDA_IO           8   
#define I2C_MASTER_NUM              I2C_NUM_0
#define I2C_MASTER_FREQ_HZ          400000
#define SSD1306_ADDR                0x3C
#define MPU6050_ADDR                0x68

#define BUZZER_GPIO                 18
#define BUTTON_NEXT_GPIO            40    
#define BUTTON_SELECT_GPIO          38     

#define DISP_W 128
#define DISP_H 64
#define DISP_TEXT_LINE_H 12

static const char *TAG = "main";

typedef enum {
    BTN_EVENT_NEXT = 1,
    BTN_EVENT_SELECT = 2,
} btn_event_t;

static QueueHandle_t btn_evt_queue;
static bool sd_mounted = false;

static void i2c_master_init(void)
{
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = I2C_MASTER_SDA_IO,
        .sda_pullup_en = GPIO_PULLUP_ENABLE,
        .scl_io_num = I2C_MASTER_SCL_IO,
        .scl_pullup_en = GPIO_PULLUP_ENABLE,
        .master.clk_speed = I2C_MASTER_FREQ_HZ,
    };
    ESP_ERROR_CHECK(i2c_param_config(I2C_MASTER_NUM, &conf));
    ESP_ERROR_CHECK(i2c_driver_install(I2C_MASTER_NUM, conf.mode, 0, 0, 0));
}

static void i2c_scan(void)
{
    ESP_LOGI(TAG, "I2C scanning...");
    for (int addr = 1; addr < 128; addr++) {
        i2c_cmd_handle_t cmd = i2c_cmd_link_create();
        i2c_master_start(cmd);
        i2c_master_write_byte(cmd, (addr << 1) | I2C_MASTER_WRITE, true);
        i2c_master_stop(cmd);
        esp_err_t ret = i2c_master_cmd_begin(I2C_MASTER_NUM, cmd, pdMS_TO_TICKS(50));
        i2c_cmd_link_delete(cmd);
        if (ret == ESP_OK) {
            ESP_LOGI(TAG, "I2C device found at 0x%02X", addr);
        }
    }
    ESP_LOGI(TAG, "I2C scan done");
}

static void buzzer_init(void)
{
    ledc_timer_config_t ledc_timer = {
        .speed_mode       = LEDC_LOW_SPEED_MODE,
        .duty_resolution  = LEDC_TIMER_8_BIT,
        .timer_num        = LEDC_TIMER_0,
        .freq_hz          = 2000,
        .clk_cfg          = LEDC_AUTO_CLK
    };
    ledc_timer_config(&ledc_timer);

    ledc_channel_config_t ledc_channel = {
        .gpio_num   = BUZZER_GPIO,
        .speed_mode = LEDC_LOW_SPEED_MODE,
        .channel    = LEDC_CHANNEL_0,
        .intr_type  = LEDC_INTR_DISABLE,
        .timer_sel  = LEDC_TIMER_0,
        .duty       = 0,
        .hpoint     = 0
    };
    ledc_channel_config(&ledc_channel);
}

static void buzzer_beep(int ms)
{
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 128);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
    vTaskDelay(pdMS_TO_TICKS(ms));
    ledc_set_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0, 0);
    ledc_update_duty(LEDC_LOW_SPEED_MODE, LEDC_CHANNEL_0);
}

static void IRAM_ATTR gpio_isr_handler(void* arg)
{
    uint32_t gpio_num = (uint32_t) arg;
    uint32_t evt = 0;
    if (gpio_num == BUTTON_NEXT_GPIO) evt = BTN_EVENT_NEXT;
    else if (gpio_num == BUTTON_SELECT_GPIO) evt = BTN_EVENT_SELECT;
    if (evt && btn_evt_queue) {
        xQueueSendFromISR(btn_evt_queue, &evt, NULL);
    }
}

static void buttons_init(void)
{
    gpio_config_t io_conf = {
        .intr_type = GPIO_INTR_NEGEDGE,
        .mode = GPIO_MODE_INPUT,
        .pin_bit_mask = (1ULL<<BUTTON_NEXT_GPIO) | (1ULL<<BUTTON_SELECT_GPIO),
        .pull_up_en = GPIO_PULLUP_ENABLE,
        .pull_down_en = GPIO_PULLDOWN_DISABLE,
    };
    gpio_config(&io_conf);

    btn_evt_queue = xQueueCreate(10, sizeof(uint32_t));
    gpio_install_isr_service(0);
    gpio_isr_handler_add(BUTTON_NEXT_GPIO, gpio_isr_handler, (void*) BUTTON_NEXT_GPIO);
    gpio_isr_handler_add(BUTTON_SELECT_GPIO, gpio_isr_handler, (void*) BUTTON_SELECT_GPIO);
}

static sdmmc_card_t* sd_card = NULL;

static esp_err_t mount_sdcard(void)
{
    if (sd_mounted) return ESP_OK;
    esp_vfs_fat_sdmmc_mount_config_t mount_config = {
        .format_if_mount_failed = false,
        .max_files = 5,
        .allocation_unit_size = 16 * 1024
    };
    sdmmc_host_t host = SDMMC_HOST_DEFAULT();
    sdmmc_slot_config_t slot_config = SDMMC_SLOT_CONFIG_DEFAULT();
    esp_err_t ret = esp_vfs_fat_sdmmc_mount("/sdcard", &host, &slot_config, &mount_config, &sd_card);
    if (ret != ESP_OK) {
        ESP_LOGW(TAG, "Failed to mount SD card (%d)", ret);
        sd_mounted = false;
        return ret;
    }
    sd_mounted = true;
    ESP_LOGI(TAG, "SD card mounted at /sdcard");
    return ESP_OK;
}

static int save_and_get_highscore(const char *filename, int score)
{
    if (mount_sdcard() != ESP_OK) {
        ESP_LOGW(TAG, "SD not mounted, cannot save score");
        return score; // still return current score as fallback
    }

    char path[128];
    snprintf(path, sizeof(path), "/sdcard/%s", filename);

    int best = 0;
    FILE *f = fopen(path, "r");
    if (f) {
        if (fscanf(f, "%d", &best) != 1) best = 0;
        fclose(f);
    }
    if (score > best) {
        best = score;
        FILE *fw = fopen(path, "w");
        if (fw) {
            fprintf(fw, "%d\n", best);
            fclose(fw);
        } else {
            ESP_LOGW(TAG, "Failed to open %s for writing", path);
        }
    }
    return best;
}

static float acce_offset_x = 0.0f;
static float acce_offset_y = 0.0f;
static float acce_offset_z = 0.0f;

static void mpu_calibrate(mpu6050_handle_t mpu)
{
    const int samples = 100;
    mpu6050_acce_value_t a;
    float sx = 0, sy = 0, sz = 0;
    for (int i = 0; i < samples; i++) {
        if (mpu6050_get_acce(mpu, &a) == ESP_OK) {
            sx += a.acce_x;
            sy += a.acce_y;
            sz += a.acce_z;
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
    acce_offset_x = sx / samples;
    acce_offset_y = sy / samples;
    acce_offset_z = sz / samples;
    ESP_LOGI(TAG, "Calibration offsets: X=%.3f Y=%.3f Z=%.3f", acce_offset_x, acce_offset_y, acce_offset_z);
}

static int accel_to_pixel_x(float raw_ax, int prev_px)
{
    const float amp = 60.0f;
    float ax = raw_ax - acce_offset_x;
    float pxf = (DISP_W / 2.0f) + ax * amp;
    if (pxf < 0) pxf = 0;
    if (pxf > DISP_W - 1) pxf = DISP_W - 1;
    float alpha = 0.30f;
    float smoothed = prev_px * (1.0f - alpha) + pxf * alpha;
    return (int)(smoothed + 0.5f);
}

/* --- Menu drawing --- */
static void draw_menu(ssd1306_handle_t display, int sel)
{
    const char *items[4] = {
        "1. Dodge Blocks",
        "2. Tilt Maze",
        "3. Snake Tilt",
        "4. Paddle Pong"
    };
    ssd1306_clear_screen(display, 0x00);
    for (int i = 0; i < 4; i++) {
        char line[32];
        if (i == sel) snprintf(line, sizeof(line), ">%s", items[i]);
        else snprintf(line, sizeof(line), " %s", items[i]);
        ssd1306_draw_string(display, 0, i * DISP_TEXT_LINE_H, (const uint8_t *)line, 12, 1);
    }
    ssd1306_refresh_gram(display);
}


static int game_dodge(ssd1306_handle_t display, mpu6050_handle_t mpu, QueueHandle_t q)
{
    #define MAX_OBS 6
    typedef struct { int x,y; bool alive; } obstacle_t;
    obstacle_t obs[MAX_OBS];
    for (int i=0;i<MAX_OBS;i++) obs[i].alive = false;
    int player_x = DISP_W/2;
    int player_y = 56;
    int prev_px = player_x;
    mpu6050_acce_value_t ac;
    uint32_t evt;
    int spawn_timer = 0;
    int score = 0;

    ssd1306_clear_screen(display, 0x00);
    ssd1306_draw_string(display, 0, 0, (const uint8_t *)"DODGE - tilt to move", 12, 1);
    ssd1306_draw_string(display, 0, 12, (const uint8_t *)"SEL->back", 12, 1);
    ssd1306_refresh_gram(display);
    vTaskDelay(pdMS_TO_TICKS(500));

    while (1) {
        if (xQueueReceive(q, &evt, 0) == pdTRUE) {
            if (evt == BTN_EVENT_SELECT) { buzzer_beep(50); break; }
        }
        if (mpu6050_get_acce(mpu, &ac) == ESP_OK) {
            player_x = accel_to_pixel_x(ac.acce_x, prev_px);
            prev_px = player_x;
        }
        spawn_timer++;
        if (spawn_timer > 6) {
            spawn_timer = 0;
            for (int i=0;i<MAX_OBS;i++) {
                if (!obs[i].alive) {
                    obs[i].alive = true;
                    obs[i].x = rand() % (DISP_W - 8);
                    obs[i].y = 16;
                    break;
                }
            }
        }
        for (int i=0;i<MAX_OBS;i++) {
            if (obs[i].alive) {
                obs[i].y += 5;

                if (obs[i].y > player_y + 8) {
                    score += 1;
                    obs[i].alive = false;
                    continue;
                }

                if (obs[i].y >= player_y - 4 && obs[i].alive) {
                    if (abs(obs[i].x - player_x) < 8) {
                        ssd1306_clear_screen(display, 0x00);
                        ssd1306_draw_string(display, 0, 0, (const uint8_t *)"Game Over!", 16, 1);
                        char sb[32];
                        snprintf(sb, sizeof(sb), "Score:%d", score);
                        ssd1306_draw_string(display, 0, 20, (const uint8_t *)sb, 12, 1);
                        ssd1306_refresh_gram(display);
                        buzzer_beep(200);
                        vTaskDelay(pdMS_TO_TICKS(1000));
                        return score;
                    }
                }

                if (obs[i].y > DISP_H + 8) {
                    obs[i].alive = false;
                    continue;
                }
            }
        }
        ssd1306_clear_screen(display, 0x00);
        char sc[32];
        snprintf(sc, sizeof(sc), "Score:%d", score);
        ssd1306_draw_string(display, 0, 0, (const uint8_t *)sc, 12, 1);
        for (int i=0;i<MAX_OBS;i++) {
            if (obs[i].alive) {
                ssd1306_draw_string(display, obs[i].x, obs[i].y, (const uint8_t *)"X", 12, 1);
            }
        }
        ssd1306_draw_string(display, player_x, player_y, (const uint8_t *)"U", 12, 1);
        ssd1306_refresh_gram(display);
        vTaskDelay(pdMS_TO_TICKS(80));
    }
    return score;
}

static inline float clampf(float v, float a, float b) {
    if (v < a) return a;
    if (v > b) return b;
    return v;
}

/******************************************************************************
 * TILT MAZE GAME 
 *****************************************************************************/

#define MAZE_COLS 16
#define MAZE_ROWS 6
#define NUM_MAZE_LEVELS 3

const int ui_top_maze = 16;
const int cell_w = 8;
const int cell_h = 8;

typedef struct {
    const uint8_t layout[MAZE_ROWS][MAZE_COLS];
    int start_r, start_c;
    int key_r, key_c;
    int goal_r, goal_c;
    int door_r, door_c;
} maze_level_t;

const maze_level_t maze_levels[NUM_MAZE_LEVELS] = {
    { // Level 1
        .layout = {
            {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
            {1,0,0,0,1,0,0,0,0,0,0,1,0,0,0,1},
            {1,0,1,0,1,0,1,1,1,1,0,1,0,1,0,1},
            {1,0,1,0,0,0,1,0,0,0,0,1,0,1,0,1},
            {1,0,1,1,1,1,1,0,1,1,1,1,0,1,0,1},
            {1,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1},
        },
        .start_r = 5, .start_c = 1,
        .key_r = 1, .key_c = 2,
        .goal_r = 1, .goal_c = 14,
        .door_r = 0, .door_c = 8,
    },
    { // Level 2
        .layout = {
            {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
            {1,0,1,0,0,0,0,1,0,0,0,0,0,0,0,1},
            {1,0,1,0,1,1,0,1,0,1,1,1,1,1,0,1},
            {1,0,0,0,1,0,0,0,0,1,0,0,0,1,0,1},
            {1,1,1,1,1,0,1,1,1,1,0,1,0,1,0,1},
            {1,0,0,0,0,0,1,0,0,0,0,1,0,0,0,1},
        },
        .start_r = 5, .start_c = 1,
        .key_r = 5, .key_c = 14,
        .goal_r = 1, .goal_c = 8,
        .door_r = 0, .door_c = 4,
    },
    { // Level 3
        .layout = {
            {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1},
            {1,0,0,0,0,1,0,0,0,1,0,0,0,0,0,1},
            {1,0,1,1,0,1,0,1,0,1,0,1,1,1,0,1},
            {1,0,1,0,0,1,0,1,0,0,0,0,0,1,0,1},
            {1,0,1,0,1,1,1,1,1,1,1,1,0,1,0,1},
            {1,0,1,0,0,0,0,0,0,0,0,0,0,1,0,1},
        },
        .start_r = 5, .start_c = 1,
        .key_r = 1, .key_c = 14,
        .goal_r = 3, .goal_c = 8,
        .door_r = 0, .door_c = 2,
    }
};


static bool check_wall_collision(float cx, float cy, float radius, const uint8_t maze[MAZE_ROWS][MAZE_COLS],
                                 bool door_open, int door_r, int door_c)
{
    int min_c = (int)floorf((cx - radius) / (float)cell_w);
    int max_c = (int)floorf((cx + radius) / (float)cell_w);
    int min_r = (int)floorf(((cy - radius) - ui_top_maze) / (float)cell_h);
    int max_r = (int)floorf(((cy + radius) - ui_top_maze) / (float)cell_h);

    if (min_c < 0) min_c = 0;
    if (max_c >= MAZE_COLS) max_c = MAZE_COLS - 1;
    if (min_r < 0) min_r = 0;
    if (max_r >= MAZE_ROWS) max_r = MAZE_ROWS - 1;

    for (int r = min_r; r <= max_r; ++r) {
        for (int c = min_c; c <= max_c; ++c) {
            uint8_t tile = maze[r][c];

            if (r == door_r && c == door_c && !door_open) {
                tile = 1;
            }
            
            if (tile == 1) { 
                float rect_x = c * cell_w;
                float rect_y = ui_top_maze + r * cell_h;
                float closest_x = clampf(cx, rect_x, rect_x + cell_w);
                float closest_y = clampf(cy, rect_y, rect_y + cell_h);
                float dx = cx - closest_x;
                float dy = cy - closest_y;
                if ((dx*dx + dy*dy) < (radius * radius)) return true;
            }
        }
    }
    return false;
}

static int run_maze_level(ssd1306_handle_t display, mpu6050_handle_t mpu, QueueHandle_t q, const maze_level_t* level, int level_num)
{
    uint8_t maze[MAZE_ROWS][MAZE_COLS];
    memcpy(maze, level->layout, sizeof(maze));

    maze[level->key_r][level->key_c] = 3;
    maze[level->goal_r][level->goal_c] = 2;
    bool door_open = false;

    float ball_x = level->start_c * cell_w + cell_w/2.0f;
    float ball_y = ui_top_maze + level->start_r * cell_h + cell_h/2.0f;
    float vx = 0.0f, vy = 0.0f;
    
    const float accel_scale = 120.0f;
    const float damping = 0.92f;
    const float vmax = 6.0f;
    const float dt = 0.06f;
    const float radius = 3.0f;

    mpu6050_acce_value_t ac;
    uint32_t evt;
    
    TickType_t start_tick = xTaskGetTickCount();

    while (1) {
        if (xQueueReceive(q, &evt, 0) == pdTRUE) {
            if (evt == BTN_EVENT_SELECT) { buzzer_beep(50); return 0; } // Quit
        }

        if (mpu6050_get_acce(mpu, &ac) == ESP_OK) {
            vx += (ac.acce_x - acce_offset_x) * accel_scale * dt;
            vy += (ac.acce_y - acce_offset_y) * accel_scale * dt;
        }

        vx *= damping; vy *= damping;
        vx = clampf(vx, -vmax, vmax);
        vy = clampf(vy, -vmax, vmax);

        float next_x = ball_x + vx;
        if (!check_wall_collision(next_x, ball_y, radius, maze, door_open, level->door_r, level->door_c)) {
            ball_x = next_x;
        } else {
            vx = 0;
        }

        float next_y = ball_y + vy;
        if (!check_wall_collision(ball_x, next_y, radius, maze, door_open, level->door_r, level->door_c)) {
            ball_y = next_y;
        } else {
            vy = 0;
        }
        
        ball_x = clampf(ball_x, radius, DISP_W - radius);
        ball_y = clampf(ball_y, ui_top_maze + radius, DISP_H - radius);

        int cur_c = (int)(ball_x / cell_w);
        int cur_r = (int)((ball_y - ui_top_maze) / cell_h);
        
        if (cur_c < 0) cur_c = 0;
        if (cur_c >= MAZE_COLS) cur_c = MAZE_COLS - 1;
        if (cur_r < 0) cur_r = 0;
        if (cur_r >= MAZE_ROWS) cur_r = MAZE_ROWS - 1;

        if (maze[cur_r][cur_c] == 3) { // Collected Key
            maze[cur_r][cur_c] = 0;
            door_open = true;
            buzzer_beep(120);
        }

        if (maze[cur_r][cur_c] == 2 || (cur_r == level->door_r && cur_c == level->door_c && door_open)) {
            uint32_t duration_ms = (xTaskGetTickCount() - start_tick) * portTICK_PERIOD_MS;
            int score = 10000 - (duration_ms / 10);
            if (score < 0) score = 0;

            ssd1306_clear_screen(display, 0x00);
            char buf[32];
            snprintf(buf, sizeof(buf), "Level %d Clear!", level_num);
            ssd1306_draw_string(display, 0, 8, (const uint8_t *)buf, 16, 1);
            snprintf(buf, sizeof(buf), "Time: %.2fs", duration_ms/1000.0f);
            ssd1306_draw_string(display, 0, 28, (const uint8_t *)buf, 12, 1);
            snprintf(buf, sizeof(buf), "Score: %d", score);
            ssd1306_draw_string(display, 0, 44, (const uint8_t *)buf, 12, 1);
            ssd1306_refresh_gram(display);
            buzzer_beep(300);
            vTaskDelay(pdMS_TO_TICKS(2500));
            return score;
        }

        ssd1306_clear_screen(display, 0x00);
        char ui_buf[40];
        uint32_t time_ms = (xTaskGetTickCount() - start_tick) * portTICK_PERIOD_MS;
        snprintf(ui_buf, sizeof(ui_buf), "Lvl:%d Time:%.1fs", level_num, time_ms/1000.0f);
        ssd1306_draw_string(display, 0, 0, (const uint8_t *)ui_buf, 12, 1);

        for (int r = 0; r < MAZE_ROWS; ++r) {
            for (int c = 0; c < MAZE_COLS; ++c) {
                int x = c * cell_w;
                int y = ui_top_maze + r * cell_h;
                uint8_t tile = maze[r][c];

                if (r == level->door_r && c == level->door_c) {
                    ssd1306_draw_string(display, x, y, (const uint8_t *)(door_open ? "O" : "D"), 8, 1);
                } else if (tile == 1) {
                    ssd1306_draw_string(display, x, y, (const uint8_t *)"#", 8, 1);
                } else if (tile == 2) {
                    ssd1306_draw_string(display, x, y, (const uint8_t *)"G", 8, 1);
                } else if (tile == 3) {
                    ssd1306_draw_string(display, x, y, (const uint8_t *)"K", 8, 1);
                }
            }
        }
        ssd1306_draw_string(display, (uint8_t)(ball_x - 3), (uint8_t)(ball_y - 4), (const uint8_t *)"o", 12, 1);
        ssd1306_refresh_gram(display);
        vTaskDelay(pdMS_TO_TICKS((int)(dt*1000)));
    }
    return 0; 
}


/******************************************************************************
 * SNAKE TILT GAME
 *****************************************************************************/
#define SNAKE_MAX 200
const int grid_step = 4;
const int ui_top_snake = 16;
const int snake_hitbox_size = 8; 

static void snake_spawn_food(int *food_x, int *food_y, int snake_x[], int snake_y[], int length) {
    bool on_snake;
    do {
        on_snake = false;
        
        int max_x_steps = (DISP_W - snake_hitbox_size) / grid_step;
        int max_y_steps = (DISP_H - ui_top_snake - snake_hitbox_size) / grid_step;

        *food_x = (rand() % (max_x_steps + 1)) * grid_step;
        *food_y = ui_top_snake + (rand() % (max_y_steps + 1)) * grid_step;

        for (int i = 0; i < length; ++i) {
            if (snake_x[i] < *food_x + snake_hitbox_size &&
                snake_x[i] + snake_hitbox_size > *food_x &&
                snake_y[i] < *food_y + snake_hitbox_size &&
                snake_y[i] + snake_hitbox_size > *food_y)
            {
                on_snake = true;
                break;
            }
        }
    } while (on_snake);
}

static int game_snake_tilt(ssd1306_handle_t display, mpu6050_handle_t mpu, QueueHandle_t q)
{
    int snake_x[SNAKE_MAX], snake_y[SNAKE_MAX];
    int length = 5;
    int food_x, food_y;
    int score = 0;

    int start_x = (DISP_W/2 / grid_step) * grid_step;
    int start_y = (DISP_H/2 / grid_step) * grid_step;
    for (int i=0;i<length;i++) {
        snake_x[i] = start_x - i * grid_step;
        snake_y[i] = start_y;
    }

    snake_spawn_food(&food_x, &food_y, snake_x, snake_y, length);
    mpu6050_acce_value_t ac;
    uint32_t evt;
    const float turn_threshold = 0.15f;
    int dir_x = 1, dir_y = 0;
    int grow_pending = 0;

    ssd1306_clear_screen(display, 0x00);
    ssd1306_draw_string(display, 0, 0, (const uint8_t *)"SNAKE - Infinity!", 12, 1);
    vTaskDelay(pdMS_TO_TICKS(1000));

    while (1) {
        if (xQueueReceive(q, &evt, 0) == pdTRUE) {
            if (evt == BTN_EVENT_SELECT) { buzzer_beep(50); break; }
        }

        if (mpu6050_get_acce(mpu, &ac) == ESP_OK) {
            float ax = ac.acce_x - acce_offset_x;
            float ay = ac.acce_y - acce_offset_y;

            if (fabsf(ax) > fabsf(ay) && fabsf(ax) > turn_threshold) {
                if (dir_x == 0) {
                    dir_x = (ax > 0) ? 1 : -1;
                    dir_y = 0;
                }
            } else if (fabsf(ay) > fabsf(ax) && fabsf(ay) > turn_threshold) {
                 if (dir_y == 0) {
                    dir_y = (ay > 0) ? 1 : -1;
                    dir_x = 0;
                }
            }
        }

        int new_head_x = snake_x[0] + dir_x * grid_step;
        int new_head_y = snake_y[0] + dir_y * grid_step;

        if (new_head_x < 0) {
            new_head_x = DISP_W - grid_step;
        } else if (new_head_x >= DISP_W) {
            new_head_x = 0;
        }

        if (new_head_y < ui_top_snake) {
            new_head_y = DISP_H - grid_step;
        } else if (new_head_y >= DISP_H) {
            new_head_y = ui_top_snake;
        }

        for (int i = 1; i < length; ++i) { 
            if (new_head_x == snake_x[i] && new_head_y == snake_y[i]) { 
                goto game_over; 
            } 
        }

        if (new_head_x < food_x + snake_hitbox_size &&
            new_head_x + snake_hitbox_size > food_x &&
            new_head_y < food_y + snake_hitbox_size &&
            new_head_y + snake_hitbox_size > food_y)
        {
            score++;
            grow_pending = 1;
            buzzer_beep(40);
            snake_spawn_food(&food_x, &food_y, snake_x, snake_y, length);
        }

        int tail_x = snake_x[length - 1];
        int tail_y = snake_y[length - 1];
        for (int i = length - 1; i > 0; --i) { snake_x[i] = snake_x[i-1]; snake_y[i] = snake_y[i-1]; }
        snake_x[0] = new_head_x;
        snake_y[0] = new_head_y;

        if (grow_pending && length < SNAKE_MAX) { snake_x[length] = tail_x; snake_y[length] = tail_y; length++; grow_pending = 0; }

        ssd1306_clear_screen(display, 0x00);
        char score_buf[16];
        snprintf(score_buf, sizeof(score_buf), "Score: %d", score);
        ssd1306_draw_string(display, 0, 0, (const uint8_t *)score_buf, 12, 1);
        ssd1306_draw_string(display, food_x, food_y, (const uint8_t *)"@", 12, 1);
        for (int i = 0; i < length; ++i) { ssd1306_draw_string(display, snake_x[i], snake_y[i], (const uint8_t *)"o", 8, 1); }
        ssd1306_refresh_gram(display);
        vTaskDelay(pdMS_TO_TICKS(150));
    }

game_over:
    ssd1306_clear_screen(display, 0x00);
    ssd1306_draw_string(display, 0, 8, (const uint8_t *)"Game Over!", 16, 1);
    char buf[32];
    snprintf(buf, sizeof(buf), "Score:%d", score);
    ssd1306_draw_string(display, 0, 28, (const uint8_t *)buf, 12, 1);
    ssd1306_refresh_gram(display);
    buzzer_beep(250);
    vTaskDelay(pdMS_TO_TICKS(1200));
    return score;
}

/******************************************************************************
 * PADDLE PONG GAME
 *****************************************************************************/

static int game_paddle_pong(ssd1306_handle_t display, mpu6050_handle_t mpu, QueueHandle_t q)
{
    mpu6050_acce_value_t ac;
    uint32_t evt;
    int paddle_x = DISP_W/2;
    int prev_px = paddle_x;
    const int paddle_pixel_w = 18;
    int paddle_y = 56;
    float ball_x = DISP_W/2.0f, ball_y = DISP_H/2.0f;
    float vx = 2.0f, vy = 2.0f;
    int score = 0;
    const int ui_top_pong = 16;

    ssd1306_clear_screen(display, 0x00);
    ssd1306_draw_string(display, 0, 0, (const uint8_t *)"PONG - tilt to move", 12, 1);
    vTaskDelay(pdMS_TO_TICKS(1000));

    while (1) {
        if (xQueueReceive(q, &evt, 0) == pdTRUE) {
            if (evt == BTN_EVENT_SELECT) { buzzer_beep(50); break; }
        }
        if (mpu6050_get_acce(mpu, &ac) == ESP_OK) {
            paddle_x = accel_to_pixel_x(ac.acce_x, prev_px);
            prev_px = paddle_x;
        }
        if (paddle_x < paddle_pixel_w/2) paddle_x = paddle_pixel_w/2;
        if (paddle_x > DISP_W - paddle_pixel_w/2) paddle_x = DISP_W - paddle_pixel_w/2;

        ball_x += vx; ball_y += vy;
        if (ball_x <= 1) { ball_x = 1; vx = -vx; }
        if (ball_x >= DISP_W - 2) { ball_x = DISP_W - 2; vx = -vx; }
        if (ball_y <= ui_top_pong) { ball_y = ui_top_pong; vy = -vy; }
        
        float paddle_left = paddle_x - (paddle_pixel_w / 2.0f);
        float paddle_right = paddle_x + (paddle_pixel_w / 2.0f);
        if (ball_y >= paddle_y - 3 && ball_y <= paddle_y + 4) {
            if (ball_x >= paddle_left && ball_x <= paddle_right) {
                vy = -fabsf(vy);
                float rel = (ball_x - paddle_x) / (paddle_pixel_w/2.0f);
                vx = clampf(vx + rel * 1.2f, -6.0f, 6.0f);
                score++;
                buzzer_beep(20);
                ball_y = paddle_y - 4;
            }
        }
        if (ball_y > DISP_H - 1) {
            ssd1306_clear_screen(display, 0x00);
            char sb[32];
            snprintf(sb, sizeof(sb), "Miss! Score:%d", score);
            ssd1306_draw_string(display, 0, 20, (const uint8_t *)sb, 12, 1);
            ssd1306_refresh_gram(display);
            buzzer_beep(200);
            vTaskDelay(pdMS_TO_TICKS(1000));
            return score;
        }

        ssd1306_clear_screen(display, 0x00);
        char sc[32];
        snprintf(sc, sizeof(sc), "Score:%d", score);
        ssd1306_draw_string(display, 0, 0, (const uint8_t *)sc, 12, 1);
        ssd1306_draw_string(display, (uint8_t)ball_x, (uint8_t)ball_y, (const uint8_t *)"o", 12, 1);
        
        int char_w = 6;
        int count = paddle_pixel_w / char_w;
        char paddle_str[32];
        memset(paddle_str, '=', count);
        paddle_str[count] = '\0';
        int draw_x = paddle_x - (count * char_w) / 2;
        ssd1306_draw_string(display, draw_x, paddle_y, (const uint8_t *)paddle_str, 12, 1);
        ssd1306_refresh_gram(display);
        vTaskDelay(pdMS_TO_TICKS(40));
    }
    return score;
}

void app_main(void)
{
    i2c_master_init();
    vTaskDelay(pdMS_TO_TICKS(50));
    i2c_scan();

    buzzer_init();
    buttons_init();

    ssd1306_handle_t display = ssd1306_create(I2C_MASTER_NUM, SSD1306_ADDR);
    if (!display) { ESP_LOGE(TAG, "Failed to create SSD1306 handle"); return; }
    ssd1306_init(display);
    ssd1306_clear_screen(display, 0x00);
    ssd1306_draw_string(display, 0, 0, (const uint8_t *)"Game System", 16, 1);
    ssd1306_draw_string(display, 0, 16, (const uint8_t *)"Press Next to start", 12, 1);
    ssd1306_refresh_gram(display);
    vTaskDelay(pdMS_TO_TICKS(500));

    mpu6050_handle_t mpu = mpu6050_create(I2C_MASTER_NUM, MPU6050_ADDR);
    if (!mpu) { ESP_LOGE(TAG, "Failed to create MPU6050 handle"); return; }
    mpu6050_wake_up(mpu);
    vTaskDelay(pdMS_TO_TICKS(50));
    ESP_ERROR_CHECK(mpu6050_config(mpu, ACCE_FS_2G, GYRO_FS_250DPS));

    ssd1306_clear_screen(display, 0x00);
    ssd1306_draw_string(display, 0, 0, (const uint8_t *)"Calibrando...", 16, 1);
    ssd1306_draw_string(display, 0, 18, (const uint8_t *)"Fique parado", 12, 1);
    ssd1306_refresh_gram(display);
    vTaskDelay(pdMS_TO_TICKS(500));
    mpu_calibrate(mpu);

    int menu_idx = 0;
    draw_menu(display, menu_idx);

    uint32_t evt;
    TickType_t last_press_tick = 0;
    const TickType_t debounce_ticks = pdMS_TO_TICKS(200);

    while (1) {
        if (xQueueReceive(btn_evt_queue, &evt, pdMS_TO_TICKS(100)) == pdTRUE) {
            TickType_t now = xTaskGetTickCount();
            if (now - last_press_tick < debounce_ticks) continue;
            last_press_tick = now;

            if (evt == BTN_EVENT_NEXT) {
                menu_idx = (menu_idx + 1) % 4;
                buzzer_beep(30);
                draw_menu(display, menu_idx);
            } else if (evt == BTN_EVENT_SELECT) {
                buzzer_beep(60);
                int score = 0;
                const char *fname = NULL;
                const char *title = NULL;

                switch (menu_idx) {
                    case 0: 
                        score = game_dodge(display, mpu, btn_evt_queue); 
                        fname = "dodge.score"; title = "Dodge"; 
                        break;
                    case 1: { // Tilt Maze game with level progression
                        title = "Tilt Maze";
                        fname = "tiltmaze.score";
                        int total_score = 0;
                        for (int i = 0; i < NUM_MAZE_LEVELS; i++) {
                            int level_score = run_maze_level(display, mpu, btn_evt_queue, &maze_levels[i], i + 1);
                            if (level_score > 0) {
                                total_score += level_score;
                            } else { // Player quit mid-game
                                break;
                            }
                        }
                        score = total_score;
                        break;
                    }
                    case 2: 
                        score = game_snake_tilt(display, mpu, btn_evt_queue); 
                        fname = "snake.score"; title = "Snake"; 
                        break;
                    case 3: 
                        score = game_paddle_pong(display, mpu, btn_evt_queue); 
                        fname = "pong.score"; title = "Pong"; 
                        break;
                }

                if (fname) {
                    int best = save_and_get_highscore(fname, score);
                    char buf[48];
                    snprintf(buf, sizeof(buf), "%s - Score:%d Melhor Score:%d", title, score, best);
                    ssd1306_clear_screen(display, 0x00);
                    ssd1306_draw_string(display, 0, 20, (const uint8_t *)buf, 12, 1);
                    ssd1306_refresh_gram(display);
                    vTaskDelay(pdMS_TO_TICKS(2000));
                }
                draw_menu(display, menu_idx);
            }
        }
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}