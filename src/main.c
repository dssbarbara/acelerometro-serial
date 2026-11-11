#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/drivers/sensor.h>

// Obter referência do dispositivo usando o nodelabel
static const struct device *const accel = DEVICE_DT_GET(DT_NODELABEL(mma8451q));

void main(void)
{
    struct sensor_value accel_x, accel_y, accel_z;
    int ret;
    uint32_t tempo_ms = 0;  // Contador de tempo em milissegundos

    printk("\n");
    printk("========================================\n");
    printk("  FRDM-KL25Z - Acelerometro MMA8451Q\n");
    printk("========================================\n");
    printk("I2C0: PTE24 (SCL), PTE25 (SDA)\n");
    printk("========================================\n\n");

    // Verificar se o dispositivo está pronto
    if (!device_is_ready(accel)) {
        printk("ERRO: Acelerometro nao esta pronto!\n");
        return;
    }

    printk("Acelerometro inicializado com sucesso!\n");
    printk("Iniciando leituras a cada 500ms...\n\n");
    
    // Pequeno delay antes de começar a enviar dados
    k_sleep(K_MSEC(1000));

    while (1) {
        // Solicitar leitura do sensor
        ret = sensor_sample_fetch(accel);
        if (ret) {
            printk("Erro ao ler sensor: %d\n", ret);
            k_sleep(K_MSEC(500));
            tempo_ms += 500;
            continue;
        }

        // Obter valores dos eixos X, Y e Z
        sensor_channel_get(accel, SENSOR_CHAN_ACCEL_X, &accel_x);
        sensor_channel_get(accel, SENSOR_CHAN_ACCEL_Y, &accel_y);
        sensor_channel_get(accel, SENSOR_CHAN_ACCEL_Z, &accel_z);

        // Formato: T: tempo_ms, X: valor, Y: valor, Z: valor
        printk("T: %u, X: %d.%06d, Y: %d.%06d, Z: %d.%06d\r\n", 
               tempo_ms,
               accel_x.val1, abs(accel_x.val2),
               accel_y.val1, abs(accel_y.val2),
               accel_z.val1, abs(accel_z.val2));

        // Aguardar 500ms antes da próxima leitura
        k_sleep(K_MSEC(500));
        tempo_ms += 500;
    }
}